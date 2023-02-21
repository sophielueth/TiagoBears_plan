# TiagoBears_plan
The planning package for team TiagoBears.

## Behavior

TBA

## Task
The task class handles basic functionality of the scene, like collision avoidance, basic setup tasks and torso movement.

Example usage:
```
task = Task()

task.move_torso_to(0.2) # value bounded to [0, 0.35] in m
task.add_table_collision() # so that moveit motion planning will avoid collision with the table.
```