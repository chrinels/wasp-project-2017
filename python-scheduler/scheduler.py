#! /usr/bin python3

import numpy as np
import datetime as dt

CONST_SLOT_INTERVAL = 5.0 # [sec]
CONST_SCHEDULED_SLOTS = 20 # Schedule for the next 20 slot.

CONST_VALID_TRAJECTORIES = ['WS', 'WR', 'WL', 'SS', 'SR', 'SL', 'ES', 'ER', 'EL', 'NS', 'NR', 'NL']

# WS = From West, going Straight
# EL = From East, going Left
trajectory_compatibility_graph = {  # Coming from West
                                    'WS': ['ER', 'NR', 'ES'],
                                    'WR': ['ER', 'NR', 'ES', 'SL', 'SS', 'NL', 'SR'],
                                    'WL': ['NR', 'SR', 'EL'],
                                    # Coming from South
                                    'SS': ['NR', 'WR', 'NS'],
                                    'SR': ['NR', 'WR', 'NS', 'EL', 'ES', 'WL', 'ER'],
                                    'SL': ['WR', 'ER', 'NL'],
                                    # Coming from East
                                    'ES': ['WR', 'SR', 'WS'],
                                    'ER': ['WR', 'SR', 'WS', 'NL', 'NS', 'SL', 'NR'],
                                    'EL': ['SR', 'NR', 'WL'],
                                    # Coming from North
                                    'NS': ['SR', 'ER', 'SS'],
                                    'NR': ['SR', 'ER', 'SS', 'WL', 'WS', 'EL', 'WR'],
                                    'NL': ['ER', 'WR', 'SL']}

# Slot start times relative to the CURRENT_TIME_SLOT_END
relative_slot_start_times = np.arange(CONST_SCHEDULED_SLOTS)*CONST_SLOT_INTERVAL

time_slots_table = [[]  for i in range(CONST_SCHEDULED_SLOTS)]


def assign_time_slot(access_time, planned_trajectory):
    # Find the time slot that starts just after the access time
    earliest_slot_index = 0
    for slot_index in range(CONST_SCHEDULED_SLOTS):
        if access_time > CURRENT_TIME_SLOT_END + relative_slot_start_times[slot_index]:
            earliest_slot_index = earliest_slot_index + 1
        else:
            break

    # Find the earliest available time slot that can be scheduled
    schedulable_slot_index = earliest_slot_index
    scheduling_successful = False
    while schedulable_slot_index < CONST_SCHEDULED_SLOTS:
        # Find the valid trajectories in this slot
        slot_valid_trajectories = CONST_VALID_TRAJECTORIES
        for assigned_trajectory in time_slots_table[schedulable_slot_index]:
            compatible_trajectories = trajectory_compatibility_graph[assigned_trajectory]
            updated_slot_valid_trajectories = []
            # Only keep trajectories that are compatible
            for traj in compatible_trajectories:
                if traj in slot_valid_trajectories:
                    updated_slot_valid_trajectories.append(traj)
            slot_valid_trajectories = updated_slot_valid_trajectories

        if planned_trajectory in slot_valid_trajectories:
            scheduling_successful = True
            break
        else:
            # Update slot index counter
            schedulable_slot_index = schedulable_slot_index + 1

    return (schedulable_slot_index, scheduling_successful)
