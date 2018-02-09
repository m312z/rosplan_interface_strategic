# rosplan_interface_strategic

This repository holds a package for strategic-tactical planning with rosplan. This incldes:
- The tactical action interface, which performs planning and execution as an action within a strategic plan.
- An example launch file and scripts.
- An example domain file.

## Description of example

The launch file will start two sets of rosplan planning system nodes, one for the tactical level and one for strategic level. It will start simulated action nodes and a tactical control node listening for the "complete_mission" action.

The scripts will populate the initial state, which includes three uncompleted goals, and then call the strategic planning system. The strategic plan will make use of the "complete_mission" action, which easily achieves goals without preconditions. During dispatch with action will be carried out by the tactical controller.

When the "complete_mission" action is executed,
1. The "strategic_planning" fact is remoevd from the knowledge base, disabling the "complete_mission" action.
2. The appropriate mission goal is added to the knowledge base.
3. The tactical planning system is called, including dispatch.
4. The goal is removed, and the "strategic_planning" fact added again.
5. The plan success is sent back as action feedack to the strategic level.
