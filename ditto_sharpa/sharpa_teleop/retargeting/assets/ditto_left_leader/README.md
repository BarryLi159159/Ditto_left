# ditto_left_leader

Mirror image of `../ditto_3f_leader` (index + thumb; middle finger dropped),
for the left-hand Ditto leader. Generated, not hand-edited:

    python sharpa_teleop/retargeting/mirror_urdf.py \
        sharpa_teleop/retargeting/assets/ditto_3f_leader/ditto_3f_leader.urdf \
        sharpa_teleop/retargeting/assets/ditto_left_leader/ditto_left_leader.urdf \
        --axis x --drop middle --name ditto_left_leader

Mirror plane is x = 0 (across the palm: index/middle are spread along x, the
fingers point along +y). Joint axes are flipped as pseudo-vectors so the same q
gives the mirror pose and the right-hand joint limits still apply; meshes are
reflected vertex-wise. thumb_joint_2 (the removed motor 113) stays fixed, as in
the source model; its mesh is kept because the physical bracket has the same
reach. Verified: left fingertips equal the mirror of the right ones to <1e-9 m
over 200 random poses.

Hand config: `conf/hand_config/ditto_left_leader_only.yaml` (zero_position
and direction calibrated 2026-10-30). Viewer: `viz/view_left_leader.py --left`.
