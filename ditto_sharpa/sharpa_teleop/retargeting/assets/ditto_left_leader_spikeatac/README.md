# ditto_left_leader_spikeatac

Variant of `../ditto_left_leader` (left Ditto leader, index + 3-motor thumb)
with SpikeATac fingertips. Joints, limits, brackets and fingerpads are the same
as `ditto_left_leader` (identical forward kinematics), so
`conf/hand_config/ditto_left_leader_only.yaml` applies unchanged.

Per finger, the old `*_fingertip` link is removed and replaced by:

- `<finger>_tip_adapter` (`meshes/tip_adapter.STL`, from `fingertip_adapter.STL`)
  bolted to the front face of the last bracket (`index_link_2` /
  `thumb_link_3`), the face the old fingertip used. The adapter's 3-hole
  plate (holes on a 6 mm circle) sits on the bracket's 6 mm hole circle, plate
  back face on the bracket face (x = 17.0 mm; thumb circle centered at
  y = -0.07, z = -8.0 mm in `thumb_link_3`).
- `<finger>_spikeatac` (`meshes/spikeatac.STL`, from `spikeatac_dummy.STL`)
  on the adapter tab, its 2 base holes on the tab's 2 holes (6.5 mm apart),
  on the underside of the tab.

The adapter rotation about the face normal (the hole pattern allows 45 deg
steps) is 90 deg on the index and 270 deg on the thumb, sensor under the tab:
of the 16 options this puts the SpikeATac where the old fingertip dome was
(mean surface distance ~0.9 mm; next best 2.2 mm), 7.6 mm clear of the
fingerpad.

How it was made: the parts (CAD in mm) were scaled to m and added to the
right-hand CAD export `ditto_3f_leader_9dof/ditto_3f_leader.urdf`
(2026-11-12) with fixed joints, then mirrored with

    python sharpa_teleop/retargeting/mirror_urdf.py <right_spikeatac.urdf> \
        sharpa_teleop/retargeting/assets/ditto_left_leader_spikeatac/ditto_left_leader_spikeatac.urdf \
        --axis x --drop middle --name ditto_left_leader_spikeatac

Both parts are mirror-symmetric (deviation 0.1-0.2 mm, at the sampling
resolution), so the left hand uses the same printed parts as the right.
Masses/inertias of the two parts are rough PLA estimates (SpikeATac from its
convex hull, its mesh is not closed).

Viewer: `viz/view_left_leader.py --left --spikeatac`.
