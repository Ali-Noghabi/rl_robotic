import pygame
import sys
from itertools import product

# Initialize Pygame
pygame.init()

# -----------------------------
# Constants and Configuration
# -----------------------------

# Board dimensions
ROWS, COLS = 5, 4
SQUARE_SIZE = 100
WIDTH, HEIGHT = COLS * SQUARE_SIZE, ROWS * SQUARE_SIZE
FPS = 60

# Colors
WHITE = (255, 255, 255)
GRAY = (128, 128, 128)
DARK_GRAY = (169, 169, 169)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)

# Piece Encoding
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

# Mapping from piece to image file (Ensure you have these images in a 'images' folder)
# For simplicity, we will use simple shapes instead of images in this implementation.

# Font for rendering text
FONT = pygame.font.SysFont(None, 24)

# -----------------------------
# Helper Functions
# -----------------------------

def encode_action(from_pos, to_pos):
    """
    Encode (from_pos, to_pos) to a single integer action.
    from_pos and to_pos are tuples: (row, col)
    """
    from_idx = from_pos[0] * COLS + from_pos[1]
    to_idx = to_pos[0] * COLS + to_pos[1]
    return from_idx * (ROWS * COLS) + to_idx

def decode_action(action):
    """
    Decode a single integer action to (from_pos, to_pos).
    Returns tuples: (from_pos, to_pos)
    """
    to_idx = action % (ROWS * COLS)
    from_idx = action // (ROWS * COLS)
    from_pos = (from_idx // COLS, from_idx % COLS)
    to_pos = (to_idx // COLS, to_idx % COLS)
    return from_pos, to_pos

# -----------------------------
# Micro Chess Environment
# -----------------------------

class MicroChess:
    def __init__(self):
        self.reset()

    def reset(self):
        # Initialize the board to the correct starting position
        self.board = [
            [BK, BN, BB, BR],       # Row 0
            [BP, EMPTY, EMPTY, EMPTY],  # Row 1
            [EMPTY, EMPTY, EMPTY, EMPTY],  # Row 2
            [EMPTY, EMPTY, EMPTY, WP],  # Row 3
            [WR, WB, WN, WK]        # Row 4
        ]
        self.current_turn = 1  # 1 for White, -1 for Black
        self.done = False
        self.winner = None
        self.move_count = 0
        self.selected_piece = None
        self.valid_moves = []

    def get_valid_moves(self, position):
        """
        Given a position, return all valid move positions based on piece type
        """
        piece = self.board[position[0]][position[1]]
        moves = []
        row, col = position

        if piece == EMPTY or piece * self.current_turn <= 0:
            return moves  # No piece to move or not player's piece

        if abs(piece) == WK:
            directions = [(-1, -1), (-1, 0), (-1, 1),
                          (0, -1),          (0, 1),
                          (1, -1),  (1, 0), (1, 1)]
            for dr, dc in directions:
                r, c = row + dr, col + dc
                if self.is_within_board(r, c):
                    target = self.board[r][c]
                    if target * self.current_turn <= 0:
                        moves.append((r, c))

        elif abs(piece) == WR:
            # Rook moves: horizontal and vertical
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            for dr, dc in directions:
                r, c = row + dr, col + dc
                while self.is_within_board(r, c):
                    target = self.board[r][c]
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
                    target = self.board[r][c]
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
                    target = self.board[r][c]
                    if target * self.current_turn <= 0:
                        moves.append((r, c))

        elif abs(piece) == WP:
            # Pawn moves: forward one, capture diagonally
            direction = -1 if piece > 0 else 1
            # Forward move
            r, c = row + direction, col
            if self.is_within_board(r, c) and self.board[r][c] == EMPTY:
                moves.append((r, c))
            # Captures
            for dc in [-1, 1]:
                r, c = row + direction, col + dc
                if self.is_within_board(r, c):
                    target = self.board[r][c]
                    if target * self.current_turn < 0:
                        moves.append((r, c))
        return moves

    def is_within_board(self, r, c):
        return 0 <= r < ROWS and 0 <= c < COLS

    def make_move(self, from_pos, to_pos):
        """
        Execute a move from from_pos to to_pos
        Returns a tuple (valid, message)
        """
        if self.done:
            return False, "Game is over."

        piece = self.board[from_pos[0]][from_pos[1]]
        if piece == EMPTY or piece * self.current_turn <= 0:
            return False, "No piece to move or not your turn."

        valid_moves = self.get_valid_moves(from_pos)
        if to_pos not in valid_moves:
            return False, "Invalid move."

        target_piece = self.board[to_pos[0]][to_pos[1]]
        # Execute move
        self.board[to_pos[0]][to_pos[1]] = piece
        self.board[from_pos[0]][from_pos[1]] = EMPTY

        # Check for capture
        capture = False
        if target_piece != EMPTY:
            capture = True

        # Check for promotion
        promotion = False
        if piece == WP and to_pos[0] == 0:
            self.board[to_pos[0]][to_pos[1]] = WR  # Promote to Rook
            promotion = True
        elif piece == BP and to_pos[0] == ROWS - 1:
            self.board[to_pos[0]][to_pos[1]] = BR  # Promote to Rook
            promotion = True

        # Increment move count
        self.move_count += 1

        # Check for checkmate or stalemate
        opponent = -self.current_turn
        if self.is_checkmate(opponent):
            self.done = True
            self.winner = self.current_turn
            message = "Checkmate! {} wins.".format("White" if self.current_turn == 1 else "Black")
            self.current_turn *= -1  # Switch turn for consistency
            return True, message
        elif self.is_stalemate(opponent):
            self.done = True
            self.winner = 0
            message = "Stalemate! It's a draw."
            self.current_turn *= -1
            return True, message
        elif self.move_count >= 100:
            self.done = True
            self.winner = 0
            message = "Move limit reached. It's a draw."
            self.current_turn *= -1
            return True, message
        else:
            # Switch turn
            self.current_turn *= -1
            if capture and promotion:
                message = "Piece captured and pawn promoted."
            elif capture:
                message = "Piece captured."
            elif promotion:
                message = "Pawn promoted."
            else:
                message = "Move executed."
            return True, message

    def is_checkmate(self, player):
        """
        Checkmate if the player's king is missing or player has no legal moves.
        """
        # Check if King is present
        king_present = False
        king_piece = WK if player == 1 else BK
        for row in self.board:
            if king_piece in row:
                king_present = True
                break
        if not king_present:
            return True

        # Check if player has any legal moves
        for r in range(ROWS):
            for c in range(COLS):
                piece = self.board[r][c]
                if piece * player > 0:
                    moves = self.get_valid_moves((r, c))
                    if moves:
                        return False
        return True

    def is_stalemate(self, player):
        """
        Stalemate if the player has no legal moves and is not in check.
        """
        # Check if King is present
        king_present = False
        king_piece = WK if player == 1 else BK
        for row in self.board:
            if king_piece in row:
                king_present = True
                break
        if not king_present:
            return False  # It's checkmate, not stalemate

        # Check if player has any legal moves
        for r in range(ROWS):
            for c in range(COLS):
                piece = self.board[r][c]
                if piece * player > 0:
                    moves = self.get_valid_moves((r, c))
                    if moves:
                        return False
        return True

# -----------------------------
# Pygame GUI Components
# -----------------------------

class ChessGUI:
    def __init__(self):
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT + 100))  # Extra space for messages
        pygame.display.set_caption("Micro Chess")
        self.clock = pygame.time.Clock()
        self.game = MicroChess()
        self.selected = None
        self.valid_moves = []
        self.message = ""
        self.running = True

    def draw_board(self):
        for r in range(ROWS):
            for c in range(COLS):
                rect = pygame.Rect(c * SQUARE_SIZE, r * SQUARE_SIZE, SQUARE_SIZE, SQUARE_SIZE)
                if (r + c) % 2 == 0:
                    pygame.draw.rect(self.screen, GRAY, rect)
                else:
                    pygame.draw.rect(self.screen, DARK_GRAY, rect)
                # Highlight selected square
                if self.selected == (r, c):
                    pygame.draw.rect(self.screen, BLUE, rect, 3)
                # Highlight valid moves
                if (r, c) in self.valid_moves:
                    pygame.draw.circle(self.screen, GREEN, rect.center, 15)

    def draw_pieces(self):
        for r in range(ROWS):
            for c in range(COLS):
                piece = self.game.board[r][c]
                if piece != EMPTY:
                    self.draw_piece(r, c, piece)

    def draw_piece(self, row, col, piece):
        # For simplicity, represent pieces with text
        piece_str = ""
        if piece == WK:
            piece_str = "WK"
        elif piece == WB:
            piece_str = "WB"
        elif piece == WN:
            piece_str = "WN"
        elif piece == WR:
            piece_str = "WR"
        elif piece == WP:
            piece_str = "WP"
        elif piece == BK:
            piece_str = "BK"
        elif piece == BB:
            piece_str = "BB"
        elif piece == BN:
            piece_str = "BN"
        elif piece == BR:
            piece_str = "BR"
        elif piece == BP:
            piece_str = "BP"

        # Set color based on piece ownership
        if piece > 0:
            color = WHITE
        else:
            color = BLACK

        text = FONT.render(piece_str, True, color)
        text_rect = text.get_rect(center=((col + 0.5) * SQUARE_SIZE, (row + 0.5) * SQUARE_SIZE))
        self.screen.blit(text, text_rect)

    def draw_message(self):
        # Draw message box
        rect = pygame.Rect(0, HEIGHT, WIDTH, 100)
        pygame.draw.rect(self.screen, BLACK, rect)
        # Render text
        lines = self.message.split('\n')
        for i, line in enumerate(lines):
            text = FONT.render(line, True, WHITE)
            self.screen.blit(text, (10, HEIGHT + 10 + i * 20))
        # Render turn info
        turn_text = "Turn: White" if self.game.current_turn == 1 else "Turn: Black"
        text = FONT.render(turn_text, True, WHITE)
        self.screen.blit(text, (10, HEIGHT + 60))

    def handle_click(self, pos):
        x, y = pos
        if y > HEIGHT:
            return  # Clicked outside the board
        c = x // SQUARE_SIZE
        r = y // SQUARE_SIZE
        if self.selected:
            if (r, c) in self.valid_moves:
                success, msg = self.game.make_move(self.selected, (r, c))
                if success:
                    self.message = msg
                else:
                    self.message = msg
                self.selected = None
                self.valid_moves = []
            else:
                # Select another piece
                piece = self.game.board[r][c]
                if piece * self.game.current_turn > 0:
                    self.selected = (r, c)
                    self.valid_moves = self.game.get_valid_moves((r, c))
                    self.message = ""
        else:
            # Select a piece
            piece = self.game.board[r][c]
            if piece * self.game.current_turn > 0:
                self.selected = (r, c)
                self.valid_moves = self.game.get_valid_moves((r, c))
                self.message = ""

    def run(self):
        while self.running:
            self.clock.tick(FPS)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if not self.game.done:
                        self.handle_click(pygame.mouse.get_pos())
                    else:
                        # If game is over, reset on any click
                        self.game.reset()
                        self.selected = None
                        self.valid_moves = []
                        self.message = "Game reset."

            # Draw everything
            self.screen.fill(GREEN)
            self.draw_board()
            self.draw_pieces()
            self.draw_message()

            pygame.display.flip()

        pygame.quit()
        sys.exit()

# -----------------------------
# Main Execution
# -----------------------------

if __name__ == "__main__":
    gui = ChessGUI()
    gui.run()
