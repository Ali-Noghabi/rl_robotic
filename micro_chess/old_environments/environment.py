import gym
from gym import spaces
import numpy as np
from itertools import product

# Piece encoding
EMPTY = 0
WK = 1   # White King
WB = 2   # White Bishop
WN = 3   # White Knight
WR = 4   # White Rook
WP = 5   # White Pawn
BK = -1  # Black King
BB = -2  # Black Bishop
BN = -3  # Black Knight
BR = -4  # Black Rook
BP = -5  # Black Pawn

class MicroChessEnv(gym.Env):
    """
    Custom Environment for Micro Chess (5x4 Grid)
    """
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(MicroChessEnv, self).__init__()
        
        # Define action and observation space
        # Actions: from_pos * 20 + to_pos (since 5x4=20 positions)
        self.action_space = spaces.Discrete(20 * 20)  # 400 possible actions
        
        # Observation space: 5x4 grid with piece encoding
        self.observation_space = spaces.Box(low=-5, high=5, shape=(5,4), dtype=np.int8)
        
        # Initialize the board
        self.reset()

    def reset(self):
        # Initialize the board to the correct starting position
        self.board = np.array([
            [BR, BN, BB, BK],       # Row 0
            [BP, EMPTY, EMPTY, EMPTY],  # Row 1
            [EMPTY, EMPTY, EMPTY, EMPTY],  # Row 2
            [EMPTY, EMPTY, EMPTY, WP],  # Row 3
            [WR, WB, WN, WK]        # Row 4
        ], dtype=np.int8)
        
        self.current_turn = 1  # 1 for White, -1 for Black
        self.done = False
        self.winner = None
        self.move_count = 0
        return self.board.copy()

    def step(self, action):
        if self.done:
            return self.board.copy(), 0, self.done, {}
        
        # Decode action
        from_idx = action // 20
        to_idx = action % 20
        from_pos = (from_idx // 4, from_idx % 4)  # (row, col)
        to_pos = (to_idx // 4, to_idx % 4)      # (row, col)
        
        piece = self.board[from_pos]
        target = self.board[to_pos]
        
        reward = 0
        info = {}
        
        # Check if it's the player's turn and if there is a piece to move
        if piece * self.current_turn <= 0:
            # Illegal move
            reward = -0.5
            self.done = False
            return self.board.copy(), reward, self.done, info
        
        # Validate move
        valid_moves = self.get_valid_moves(from_pos)
        if to_pos not in valid_moves:
            # Illegal move
            reward = -0.5
            self.done = False
            return self.board.copy(), reward, self.done, info
        
        # Execute move
        self.board[to_pos] = piece
        self.board[from_pos] = EMPTY
        
        # Check if a capture occurred
        if target != EMPTY:
            reward += 0.1  # Capture reward
        
        # Check for promotion (if pawn reaches last rank)
        if piece == WP and to_pos[0] == 0:
            self.board[to_pos] = WR  # Promote to Rook, for simplicity
            reward += 0.5  # Promotion reward
        elif piece == BP and to_pos[0] == 4:
            self.board[to_pos] = BR  # Promote to Rook, for simplicity
            reward += 0.5  # Promotion reward
        
        # Increment move count
        self.move_count += 1
        
        # Check for checkmate or stalemate
        if self.is_checkmate(-self.current_turn):
            self.done = True
            self.winner = self.current_turn
            reward += 1  # Winning reward
        elif self.is_stalemate(-self.current_turn):
            self.done = True
            self.winner = 0  # Draw
            reward += 0
        elif self.move_count >= 100:
            self.done = True
            self.winner = 0  # Draw due to move limit
            reward += 0
        else:
            self.done = False
        
        # Switch turn if game is not done
        if not self.done:
            self.current_turn *= -1
        
        return self.board.copy(), reward, self.done, info

    def render(self, mode='human'):
        piece_symbols = {
            EMPTY: '.',
            WK: 'K', WB: 'B', WN: 'N', WR: 'R', WP: 'P',
            BK: 'k', BB: 'b', BN: 'n', BR: 'r', BP: 'p'
        }
        print("  0 1 2 3")
        for i, row in enumerate(self.board):
            row_str = ' '.join([piece_symbols.get(x, '?') for x in row])
            print(f"{i} {row_str}")
        print(f"Turn: {'White' if self.current_turn == 1 else 'Black'}\n")

    def get_valid_moves(self, position):
        """
        Given a position, return all valid move positions based on piece type
        """
        piece = self.board[position]
        moves = []
        row, col = position
        
        if abs(piece) == WK:
            directions = [(-1, -1), (-1, 0), (-1, 1),
                          (0, -1),          (0, 1),
                          (1, -1),  (1, 0), (1, 1)]
            for dr, dc in directions:
                r, c = row + dr, col + dc
                if self.is_within_board(r, c):
                    target = self.board[r, c]
                    if target * self.current_turn <= 0:
                        moves.append((r, c))
        
        elif abs(piece) == WR:
            # Rook moves: horizontal and vertical
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            for dr, dc in directions:
                r, c = row + dr, col + dc
                while self.is_within_board(r, c):
                    target = self.board[r, c]
                    if target == EMPTY:
                        moves.append((r, c))
                    elif target * self.current_turn < 0:
                        moves.append((r, c))
                        break
                    else:
                        break
                    r += dr
                    c += dc
        
        elif abs(piece) == WB:
            # Bishop moves: diagonals
            directions = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
            for dr, dc in directions:
                r, c = row + dr, col + dc
                while self.is_within_board(r, c):
                    target = self.board[r, c]
                    if target == EMPTY:
                        moves.append((r, c))
                    elif target * self.current_turn < 0:
                        moves.append((r, c))
                        break
                    else:
                        break
                    r += dr
                    c += dc
        
        elif abs(piece) == WN:
            # Knight moves: L-shapes
            knight_moves = [(-2, -1), (-2, 1), (-1, -2), (-1, 2),
                           (1, -2), (1, 2), (2, -1), (2, 1)]
            for dr, dc in knight_moves:
                r, c = row + dr, col + dc
                if self.is_within_board(r, c):
                    target = self.board[r, c]
                    if target * self.current_turn <= 0:
                        moves.append((r, c))
        
        elif abs(piece) == WP:
            # Pawn moves: forward one, capture diagonally
            direction = -1 if piece > 0 else 1
            # Forward move
            r, c = row + direction, col
            if self.is_within_board(r, c) and self.board[r, c] == EMPTY:
                moves.append((r, c))
            # Captures
            for dc in [-1, 1]:
                r, c = row + direction, col + dc
                if self.is_within_board(r, c):
                    target = self.board[r, c]
                    if target * self.current_turn < 0:
                        moves.append((r, c))
        
        return moves

    def is_within_board(self, r, c):
        return 0 <= r < 5 and 0 <= c < 4

    def is_checkmate(self, player):
        """
        Simplified checkmate: if the player has no King or no legal moves
        """
        # Check if King is present
        king_present = False
        for row in self.board:
            if player == 1 and WK in row:
                king_present = True
                break
            elif player == -1 and BK in row:
                king_present = True
                break
        if not king_present:
            return True
        
        # Check if player has any legal moves
        for r in range(5):
            for c in range(4):
                piece = self.board[r, c]
                if piece * player > 0:
                    moves = self.get_valid_moves((r, c))
                    if moves:
                        return False
        return True

    def is_stalemate(self, player):
        """
        Stalemate: player has no legal moves and is not in check
        """
        # Check if King is present
        king_present = False
        for row in self.board:
            if player == 1 and WK in row:
                king_present = True
                break
            elif player == -1 and BK in row:
                king_present = True
                break
        if not king_present:
            return False  # It's a checkmate, not stalemate
        
        # Check if player has any legal moves
        for r in range(5):
            for c in range(4):
                piece = self.board[r, c]
                if piece * player > 0:
                    moves = self.get_valid_moves((r, c))
                    if moves:
                        return False
        return True
