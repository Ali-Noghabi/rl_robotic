# action_encoding.py

def get_all_possible_actions(board_size=(5, 4)):
    """
    Generates all possible actions for the given board size.
    Each action is a tuple: (from_row, from_col, to_row, to_col)
    """
    actions = []
    for from_row in range(board_size[0]):
        for from_col in range(board_size[1]):
            for to_row in range(board_size[0]):
                for to_col in range(board_size[1]):
                    actions.append((from_row, from_col, to_row, to_col))
    return actions

ALL_ACTIONS = get_all_possible_actions()
ACTION_TO_INT = {action: idx for idx, action in enumerate(ALL_ACTIONS)}
INT_TO_ACTION = {idx: action for idx, action in enumerate(ALL_ACTIONS)}
