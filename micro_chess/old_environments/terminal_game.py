import gym
from gym import spaces
import numpy as np
from itertools import product

# Piece encoding
# Positive numbers for White, Negative for Black
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
    Custom Environment for Micro Chess (5x4 Grid) with Terminal Interface
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
            [BK, BN, BB, BR],         # Row 0
            [BP, EMPTY, EMPTY, EMPTY],  # Row 1
            [EMPTY, EMPTY, EMPTY, EMPTY],  # Row 2
            [EMPTY, EMPTY, EMPTY, WP],  # Row 3
            [WR, WB, WN, WK]          # Row 4
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
        valid_moves = self.get_valid_moves(from_pos, player=self.current_turn)
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
        opponent = -self.current_turn
        if self.is_checkmate(opponent):
            self.done = True
            self.winner = self.current_turn
            reward += 1  # Winning reward
            info['result'] = f"{'White' if self.current_turn == 1 else 'Black'} wins by checkmate."
        elif self.is_stalemate(opponent):
            self.done = True
            self.winner = 0
            reward += 0
            info['result'] = "Stalemate! It's a draw."
        elif self.move_count >= 100:
            self.done = True
            self.winner = 0
            reward += 0
            info['result'] = "Move limit reached. It's a draw."
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
        print(f"Turn: {'White' if self.current_turn == 1 else 'Black'}")

    def get_valid_moves(self, position, player=None):
        """
        Given a position and player, return all valid move positions based on piece type.
        If player is None, defaults to self.current_turn.
        """
        if player is None:
            player = self.current_turn

        piece = self.board[position]
        moves = []
        row, col = position

        if piece == EMPTY or piece * player <= 0:
            return moves  # No piece to move or not player's piece

        if abs(piece) == WK:
            directions = [(-1, -1), (-1, 0), (-1, 1),
                        (0, -1),          (0, 1),
                        (1, -1),  (1, 0), (1, 1)]
            for dr, dc in directions:
                r, c = row + dr, col + dc
                if self.is_within_board(r, c):
                    target = self.board[r, c]
                    if target * player <= 0:
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
                    elif target * player < 0:
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
                    elif target * player < 0:
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
                    if target * player <= 0:
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
                    if target * player < 0:
                        moves.append((r, c))

        return moves

    def is_within_board(self, r, c):
        return 0 <= r < 5 and 0 <= c < 4
    
    def is_in_check(self, player):
        """
        Determine if the specified player is in check.
        """
        # Find the King's position
        king_piece = WK if player == 1 else BK
        king_pos = None
        for r in range(5):
            for c in range(4):
                if self.board[r][c] == king_piece:
                    king_pos = (r, c)
                    break
            if king_pos:
                break
        if not king_pos:
            return False  # King is captured, already handled in checkmate

        # Check all opponent pieces to see if any can capture the King
        opponent = -player
        for r in range(5):
            for c in range(4):
                piece = self.board[r][c]
                if piece * opponent > 0:
                    moves = self.get_valid_moves((r, c), player=opponent)
                    if king_pos in moves:
                        return True
        return False

    def is_checkmate(self, player):
        """
        Checkmate if the player's king is in check and the player has no legal moves.
        """
        if not self.is_in_check(player):
            return False
        # Check if player has any legal moves
        for r in range(5):
            for c in range(4):
                piece = self.board[r][c]
                if piece * player > 0:
                    moves = self.get_valid_moves((r, c), player=player)
                    for move in moves:
                        # Simulate the move
                        original_piece = self.board[move[0]][move[1]]
                        self.board[move[0]][move[1]] = piece
                        self.board[r][c] = EMPTY
                        in_check = self.is_in_check(player)
                        # Undo the move
                        self.board[r][c] = piece
                        self.board[move[0]][move[1]] = original_piece
                        if not in_check:
                            return False  # Found a legal move that doesn't leave King in check
        return True  # No legal moves to escape check

    def is_stalemate(self, player):
        """
        Stalemate if the player is not in check and has no legal moves.
        """
        if self.is_in_check(player):
            return False
        # Check if player has any legal moves
        for r in range(5):
            for c in range(4):
                piece = self.board[r][c]
                if piece * player > 0:
                    moves = self.get_valid_moves((r, c), player=player)
                    for move in moves:
                        # Simulate the move
                        original_piece = self.board[move[0]][move[1]]
                        self.board[move[0]][move[1]] = piece
                        self.board[r][c] = EMPTY
                        in_check = self.is_in_check(player)
                        # Undo the move
                        self.board[r][c] = piece
                        self.board[move[0]][move[1]] = original_piece
                        if not in_check:
                            return False  # Found a legal move
        return True  # No legal moves and not in check

    def get_all_valid_actions(self):
        """
        Get all valid actions for the current player.
        Returns a list of action integers.
        """
        actions = []
        for from_idx in range(20):
            from_pos = (from_idx // 4, from_idx % 4)
            piece = self.board[from_pos]
            if piece * self.current_turn > 0:
                moves = self.get_valid_moves(from_pos)
                for to_pos in moves:
                    to_idx = to_pos[0] * 4 + to_pos[1]
                    action = from_idx * 20 + to_idx
                    actions.append(action)
        return actions

    def render_board(self):
        """
        Render the board in a readable format in the terminal.
        """
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

# -----------------------------
# Terminal-Based Play Function
# -----------------------------

def terminal_play():
    env = MicroChessEnv()
    env.reset()
    while True:
        env.render_board()
        valid_actions = env.get_all_valid_actions()
        if not valid_actions:
            if env.is_in_check(env.current_turn):
                print(f"Checkmate! {'White' if env.current_turn == -1 else 'Black'} wins.")
            else:
                print("Stalemate! It's a draw.")
            break
        
        # List valid actions
        action_map = {}
        print("Available moves:")
        for idx, action in enumerate(valid_actions):
            from_pos, to_pos = decode_action(action)
            action_map[idx] = action
            print(f"{idx}: from {from_pos} to {to_pos}")
        
        # Get user input
        while True:
            try:
                choice = int(input(f"{'White' if env.current_turn == 1 else 'Black'}'s move - select move number: "))
                if choice in action_map:
                    selected_action = action_map[choice]
                    break
                else:
                    print("Invalid choice. Please select a valid move number.")
            except ValueError:
                print("Invalid input. Please enter a number corresponding to your chosen move.")
        
        # Apply the action
        observation, reward, done, info = env.step(selected_action)
        
        # Check if game ended
        if done:
            env.render_board()
            if env.winner == 1:
                print("White wins!")
            elif env.winner == -1:
                print("Black wins!")
            else:
                print("It's a draw!")
            if 'result' in info:
                print(info['result'])
            break
        else:
            print(info.get('result', 'Move executed.\n'))

# -----------------------------
# Action Encoding and Decoding
# -----------------------------

def encode_action(from_pos, to_pos):
    """
    Encode (from_pos, to_pos) to a single integer action.
    from_pos and to_pos are tuples: (row, col)
    """
    from_idx = from_pos[0] * 4 + from_pos[1]
    to_idx = to_pos[0] * 4 + to_pos[1]
    return from_idx * 20 + to_idx

def decode_action(action):
    """
    Decode a single integer action to (from_pos, to_pos).
    Returns tuples: (from_pos, to_pos)
    """
    to_idx = action % 20
    from_idx = action // 20
    from_pos = (from_idx // 4, from_idx % 4)
    to_pos = (to_idx // 4, to_idx % 4)
    return from_pos, to_pos

# -----------------------------
# Main Execution
# -----------------------------

if __name__ == "__main__":
    print("Welcome to Micro Chess!\n")
    terminal_play()
