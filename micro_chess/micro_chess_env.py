# micro_chess_env.py

import gym
from gym import spaces
import numpy as np
from action_encoding import get_all_possible_actions, ACTION_TO_INT, INT_TO_ACTION

# Define piece encoding
PIECE_ENCODING = {
    'K': 1,    # White King
    'N': 2,    # White Knight
    'B': 3,    # White Bishop
    'R': 4,    # White Rook
    'P': 5,    # White Pawn
    'k': -1,   # Black King
    'n': -2,   # Black Knight
    'b': -3,   # Black Bishop
    'r': -4,   # Black Rook
    'p': -5    # Black Pawn
}

# Initial board setup (5 rows x 4 columns)
INITIAL_BOARD = [
    ['K', 'N', 'B', 'R'],   # Row 0
    ['P', '*', '*', '*'],   # Row 1
    ['*', '*', '*', '*'],   # Row 2
    ['*', '*', '*', 'P'],   # Row 3
    ['r', 'b', 'n', 'k']    # Row 4
]

class MicroChessEnv(gym.Env):
    """
    Custom Environment for 5x4 Micro Chess.
    """
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(MicroChessEnv, self).__init__()
        self.board_size = (5, 4)  # 5 rows x 4 columns
        self.all_actions = get_all_possible_actions(self.board_size)
        self.action_space = spaces.Discrete(len(self.all_actions))
        self.observation_space = spaces.Box(low=-5, high=5, shape=self.board_size, dtype=np.int8)
        self.reset()

    def reset(self):
        """
        Resets the environment to the initial state.
        """
        self.board = np.zeros(self.board_size, dtype=np.int8)
        for r, row in enumerate(INITIAL_BOARD):
            for c, piece in enumerate(row):
                self.board[r, c] = PIECE_ENCODING.get(piece, 0)
        self.current_turn = 1  # 1 for White, -1 for Black
        self.done = False
        self.reward = 0
        self.info = {}
        return self._get_observation()

    def _get_observation(self):
        """
        Returns the current board state.
        """
        return self.board.copy()

    def step(self, action):
        """
        Executes the given action and updates the state.
        """
        if self.done:
            return self._get_observation(), self.reward, self.done, self.info

        # Decode action
        from_row, from_col, to_row, to_col = self._decode_action(action)

        # Validate coordinates
        if not self._is_within_board(from_row, from_col) or not self._is_within_board(to_row, to_col):
            self.reward = -10  # Penalty for invalid move
            self.done = True
            self.info = {"invalid_move": "Action out of board bounds."}
            return self._get_observation(), self.reward, self.done, self.info

        piece = self.board[from_row, from_col]
        target = self.board[to_row, to_col]

        # Ensure it's the current player's piece
        if piece * self.current_turn <= 0:
            self.reward = -10  # Penalty for moving opponent's piece or no piece
            self.done = True
            self.info = {"invalid_move": "Attempted to move invalid or opponent's piece."}
            return self._get_observation(), self.reward, self.done, self.info

        # Validate move based on piece type
        if not self._is_valid_move(piece, from_row, from_col, to_row, to_col):
            self.reward = -10  # Penalty for invalid move
            self.done = True
            self.info = {"invalid_move": "Move does not follow piece movement rules."}
            return self._get_observation(), self.reward, self.done, self.info

        # Move piece
        self.board[to_row, to_col] = piece
        self.board[from_row, from_col] = 0

        # Check for capture
        if target != 0:
            if target == -1 * piece:  # Capturing opponent's piece
                if target == -1 or target == 1:  # Capturing King
                    self.done = True
                    self.reward = 1 * self.current_turn * 60000  # Large reward for capturing King
                else:
                    self.reward = 1 * self.current_turn * 300  # Reward for capturing other pieces
        else:
            self.reward = 0  # No capture

        # Switch turn
        self.current_turn *= -1

        return self._get_observation(), self.reward, self.done, self.info

    def _decode_action(self, action):
        """
        Decodes the integer action to move coordinates.
        """
        return self.all_actions[action]

    def _is_within_board(self, row, col):
        """
        Checks if the given coordinates are within the board.
        """
        return 0 <= row < self.board_size[0] and 0 <= col < self.board_size[1]

    def _is_valid_move(self, piece, from_row, from_col, to_row, to_col):
        """
        Validates the move based on piece type.
        """
        dr = to_row - from_row
        dc = to_col - from_col

        if piece == 1 or piece == -1:  # King
            return max(abs(dr), abs(dc)) == 1

        elif piece == 2 or piece == -2:  # Knight
            return (abs(dr), abs(dc)) in [(2, 1), (1, 2)]

        elif piece == 3 or piece == -3:  # Bishop
            if abs(dr) == abs(dc) and dr != 0:
                return self._is_path_clear(from_row, from_col, to_row, to_col)
            return False

        elif piece == 4 or piece == -4:  # Rook
            if (dr == 0 and dc != 0) or (dc == 0 and dr != 0):
                return self._is_path_clear(from_row, from_col, to_row, to_col)
            return False

        elif piece == 5 or piece == -5:  # Pawn
            direction = -1 if piece > 0 else 1  # White pawns move up, Black pawns move down
            # Simple pawn move: one step forward
            if dr == direction and dc == 0 and self.board[to_row, to_col] == 0:
                return True
            # Pawn capture
            if dr == direction and abs(dc) == 1 and self.board[to_row, to_col] * piece < 0:
                return True
            return False

        return False

    def _is_path_clear(self, from_row, from_col, to_row, to_col):
        """
        Checks if the path between source and destination is clear for sliding pieces.
        """
        dr = to_row - from_row
        dc = to_col - from_col
        step_r = np.sign(dr)
        step_c = np.sign(dc)
        current_r, current_c = from_row + step_r, from_col + step_c
        while (current_r, current_c) != (to_row, to_col):
            if self.board[current_r, current_c] != 0:
                return False
            current_r += step_r
            current_c += step_c
        return True

    def get_legal_actions(self):
        """
        Returns a list of all legal actions for the current player.
        """
        legal_actions = []
        for action in self.all_actions:
            from_r, from_c, to_r, to_c = action
            piece = self.board[from_r, from_c]
            if piece * self.current_turn <= 0:
                continue
            if self._is_valid_move(piece, from_r, from_c, to_r, to_c):
                legal_actions.append(action)
        return legal_actions

    def render(self, mode='human'):
        """
        Renders the current board state.
        """
        piece_map = {v: k for k, v in PIECE_ENCODING.items()}
        print("  " + " ".join([str(c) for c in range(self.board_size[1])]))
        for r in range(self.board_size[0]):
            row = [piece_map.get(self.board[r, c], '*') for c in range(self.board_size[1])]
            print(f"{r} " + " ".join(row))
        print()
