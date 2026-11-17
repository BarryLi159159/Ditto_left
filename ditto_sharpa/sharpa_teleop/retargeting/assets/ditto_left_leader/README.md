# ditto_left_leader

Mirror image of the right-hand 3-motor-thumb Ditto leader CAD export
(`ditto_3f_leader_9dof`, URDF `ditto_3f_leader.urdf`, 2026-11-12), with the
middle finger dropped (index + thumb), for the left-hand Ditto leader.
Generated, not hand-edited:

    python sharpa_teleop/retargeting/mirror_urdf.py \
        ditto_3f_leader_9dof/ditto_3f_leader.urdf \
        sharpa_teleop/retargeting/assets/ditto_left_leader/ditto_left_leader.urdf \
        --axis x --drop middle --name ditto_left_leader

Mirror plane is x = 0 (across the palm: index/middle are spread along x, the
fingers point along +y). Joint axes are flipped as pseudo-vectors so the same q
gives the mirror pose and the right-hand joint limits still apply; meshes are
reflected vertex-wise.

The thumb has three motors and three revolute joints: thumb_joint_0 (111),
thumb_joint_1 (112), thumb_joint_3 (114). The previous model drew a fourth,
fixed thumb segment (thumb_joint_2 / thumb_link_2, the removed motor 113); it
is gone here. The kinematics are unchanged: index and thumb fingerpads match
the previous model to <0.01 mm over 200 random poses, so the existing hand
config calibration still applies. New fixed frames index_fingertip and
thumb_fingertip come from the CAD export. Verified: left fingertips equal the
mirror of the right ones to <1e-9 m over 200 random poses.

Hand config: `conf/hand_config/ditto_left_leader_only.yaml` (zero_position
and direction calibrated 2026-10-30). Viewer: `viz/view_left_leader.py --left`.
