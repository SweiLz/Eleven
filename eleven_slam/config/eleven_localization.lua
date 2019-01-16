include "eleven_robot.lua"

TRAJECTORY_BUILDER.pure_localization = true
-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--     max_submaps_to_keep = 0,
-- }
POSE_GRAPH.optimize_every_n_nodes = 1

return options

