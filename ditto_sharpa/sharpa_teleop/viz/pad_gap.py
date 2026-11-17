"""Live index-thumb gap readout for the left Ditto leader viewer.

Shows, in a viser GUI folder, updated ~5x per second from the model pose:
  - closest gap between the printed distal parts of the index
    (index_link_2 + index_fingerpad + index_fingertip, or tip adapter +
    SpikeATac) and of the thumb (same parts on thumb_link_3) mesh surfaces, in mm.
    This is the number to compare with a caliper on the real glove.
  - distance between the index_fingerpad and thumb_fingerpad frame origins and
    between the *_fingertip frame origins (frames are not on the surfaces, so
    these never reach 0; use them as repeatable references).
A "Log this pose" button appends the joint angles, the model numbers and an
optional caliper-measured gap to pad_gap_log.csv (next to this package), for
fitting link lengths later.

Gap method: a coarse pass (2 mm surface samples, KD-trees) gives an upper
bound; within REFINE_BELOW_M only thumb samples near that bound go through a
dense pass (0.5 mm samples) and an exact projection onto the index meshes.
Farther apart, the coarse value is shown (good to ~1 mm). The distance is
unsigned: overlapping parts read as ~0, not negative.

Usage (done by view_left_leader.py --left):
    import viz.pad_gap as PG
    PG.install(get_server=lambda: server_or_None, urdf_path=LEFT_URDF)
"""
from __future__ import annotations

import csv
import datetime
import threading
import time
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree
from viser.extras import ViserUrdf

# Distal parts per finger; links missing from a model are skipped (the
# SpikeATac variant has *_tip_adapter + *_spikeatac instead of *_fingertip).
INDEX_LINKS = ("index_link_2", "index_fingerpad", "index_fingertip", "index_tip_adapter", "index_spikeatac")
THUMB_LINKS = ("thumb_link_3", "thumb_fingerpad", "thumb_fingertip", "thumb_tip_adapter", "thumb_spikeatac")
SAMPLE_SPACING_M = 0.0005  # dense surface sample spacing
COARSE_SPACING_M = 0.002  # coarse surface sample spacing
REFINE_BELOW_M = 0.010  # refine only when the coarse gap is below this
REFINE_K = 64  # candidate points projected exactly onto the index meshes
UPDATE_PERIOD_S = 0.2
LOG_PATH = Path(__file__).resolve().parent.parent.parent / "pad_gap_log.csv"


def _link_mesh(urdf, urdf_dir: Path, link: str) -> trimesh.Trimesh | None:
    parts = []
    for v in urdf.link_map[link].visuals:
        mesh = getattr(v.geometry, "mesh", None)
        if mesh is None:
            continue
        m = trimesh.load(urdf_dir / mesh.filename, force="mesh")
        if mesh.scale is not None:
            m.apply_scale(mesh.scale)
        if v.origin is not None:
            m.apply_transform(v.origin)
        parts.append(m)
    return trimesh.util.concatenate(parts) if parts else None


def _samples(m: trimesh.Trimesh, spacing: float, with_vertices: bool) -> np.ndarray:
    n = max(200, int(m.area / spacing**2))
    pts, _ = trimesh.sample.sample_surface(m, n, seed=0)
    return np.vstack([pts, m.vertices]) if with_vertices else pts


class _Finger:
    def __init__(self, urdf, urdf_dir: Path, links):
        self.links = []
        for link in links:
            if link not in urdf.link_map:
                continue
            m = _link_mesh(urdf, urdf_dir, link)
            if m is None:
                continue
            pts = _samples(m, SAMPLE_SPACING_M, True)
            cpts = _samples(m, COARSE_SPACING_M, False)
            self.links.append({"name": link, "mesh": m, "pts": pts, "tree": cKDTree(pts), "cpts": cpts, "ctree": cKDTree(cpts),
                               "pq": trimesh.proximity.ProximityQuery(m)})


def _transform(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return pts @ T[:3, :3].T + T[:3, 3]


class PadGap:
    def __init__(self, urdf_path: Path):
        import yourdfpy

        urdf = yourdfpy.URDF.load(str(urdf_path), load_meshes=False)
        self.urdf = urdf  # private copy for the worker thread
        d = Path(urdf_path).parent
        self.index = _Finger(urdf, d, INDEX_LINKS)
        self.thumb = _Finger(urdf, d, THUMB_LINKS)

    def gap(self, urdf) -> tuple[float, bool]:
        """(closest surface distance in m, refined?) between index and thumb distal parts."""
        Tw = {l["name"]: urdf.get_transform(l["name"]) for l in (*self.index.links, *self.thumb.links)}
        thumb_coarse = np.vstack([_transform(Tw[l["name"]], l["cpts"]) for l in self.thumb.links])
        local, coarse = {}, {}
        for il in self.index.links:
            T_inv = np.linalg.inv(Tw[il["name"]])
            local[il["name"]] = T_inv
            coarse[il["name"]] = float(il["ctree"].query(_transform(T_inv, thumb_coarse), k=1)[0].min())
        upper = min(coarse.values())  # distance between real surface points: an upper bound
        if upper >= REFINE_BELOW_M:
            return upper, False
        margin = 3 * COARSE_SPACING_M
        thumb_dense = np.vstack([_transform(Tw[l["name"]], l["pts"]) for l in self.thumb.links])
        best = upper
        for il in self.index.links:
            if coarse[il["name"]] > upper + margin:
                continue
            q = _transform(local[il["name"]], thumb_dense)  # thumb samples in this index link's frame
            near = np.isfinite(il["ctree"].query(q, k=1, distance_upper_bound=upper + margin)[0])
            q = q[near]
            if len(q) == 0:
                continue
            d, _ = il["tree"].query(q, k=1)
            k = min(REFINE_K, len(d))
            _, dist, _ = il["pq"].on_surface(q[np.argpartition(d, k - 1)[:k]])
            best = min(best, float(dist.min()))
        return best, True


def install(get_server, urdf_path: Path) -> None:
    """Patch ViserUrdf.update_cfg so the Ditto model's pose drives the readout.

    The hook only records the latest joint config; a daemon thread computes the
    gap on its own URDF copy, so the viewer loop is never slowed down.
    """
    pg = PadGap(urdf_path)
    state = {"gui": None, "pending": None, "last": None, "lock": threading.Lock()}
    orig = ViserUrdf.update_cfg

    def ensure_gui():
        server = get_server()
        if state["gui"] is not None or server is None:
            return
        with server.gui.add_folder("Index-thumb gap (model)"):
            md = server.gui.add_markdown("_waiting for pose..._")
            caliper = server.gui.add_number("Caliper gap (mm, optional)", initial_value=0.0, step=0.01)
            note = server.gui.add_text("Note", initial_value="")
            btn = server.gui.add_button("Log this pose")
        state["gui"] = md

        @btn.on_click
        def _(_):
            last = state["last"]
            if last is None:
                print("pad_gap: no pose yet", flush=True)
                return
            new = not LOG_PATH.exists()
            with LOG_PATH.open("a", newline="") as f:
                w = csv.writer(f)
                if new:
                    w.writerow(["time", *[f"{n}_deg" for n in last["names"]], "model_gap_mm", "gap_refined", "pad_frame_dist_mm", "tip_frame_dist_mm", "caliper_gap_mm", "note"])
                w.writerow([datetime.datetime.now().isoformat(timespec="seconds"), *[f"{np.rad2deg(a):.2f}" for a in last["q"]],
                            f"{last['gap']:.3f}", int(last["refined"]), f"{last['pad']:.3f}", f"{last['tip']:.3f}",
                            f"{caliper.value:.3f}" if caliper.value else "", note.value])
            print(f"pad_gap: logged to {LOG_PATH} (model gap {last['gap']:.2f} mm, caliper {caliper.value:.2f} mm)", flush=True)

    def update_cfg(self, configuration):
        orig(self, configuration)
        urdf = self._urdf
        if "index_fingerpad" in urdf.link_map and "thumb_fingerpad" in urdf.link_map:
            with state["lock"]:
                state["pending"] = (list(urdf.actuated_joint_names), np.array(urdf.cfg, dtype=float))

    def worker():
        shown = None
        while True:
            time.sleep(UPDATE_PERIOD_S)
            with state["lock"]:
                pending = state["pending"]
            if pending is None:
                continue
            ensure_gui()
            names, q = pending
            key = q.round(5).tobytes()
            if key == shown:
                continue
            try:
                u = pg.urdf
                u.update_cfg(dict(zip(names, q)))
                gap, refined = pg.gap(u)
                gap *= 1e3
                pad = np.linalg.norm(u.get_transform("index_fingerpad")[:3, 3] - u.get_transform("thumb_fingerpad")[:3, 3]) * 1e3
                tip = np.nan
                if "index_fingertip" in u.link_map and "thumb_fingertip" in u.link_map:
                    tip = np.linalg.norm(u.get_transform("index_fingertip")[:3, 3] - u.get_transform("thumb_fingertip")[:3, 3]) * 1e3
            except Exception as e:  # keep the viewer alive
                print(f"pad_gap: {e!r}", flush=True)
                continue
            shown = key
            state["last"] ={"names": names, "q": q, "gap": gap, "refined": refined, "pad": pad, "tip": tip}
            if state["gui"] is not None:
                head = f"**Closest surface gap: {gap:.2f} mm**" if refined else f"**Closest surface gap: ~{gap:.0f} mm** (coarse, >10 mm)"
                tip_line = f"  \nTip frame distance: {tip:.2f} mm" if np.isfinite(tip) else ""
                state["gui"].content = f"{head}  \nPad frame distance: {pad:.2f} mm{tip_line}"

    ViserUrdf.update_cfg = update_cfg
    threading.Thread(target=worker, name="pad_gap", daemon=True).start()
