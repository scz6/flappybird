from PIL import Image

# === CONFIG ===
IMAGE_PATH = "/Users/selenazhang/Downloads/full_pipe_colored.png"
PADDED_WIDTH = 32              # must be multiple of 16
TRANSPARENT_COLOR = (255, 0, 255)

# VGA 4-bit palette (change if needed)
PALETTE = [
    (0,0,0),       # 0 black
    (0,64,0),      # 1 dark green
    (0,128,0),     # 2 med green
    (0,255,0),     # 3 green
    (0,0,64),      # 4 dark blue
    (0,0,255),     # 5 blue
    (0,128,255),   # 6 light blue
    (0,255,255),   # 7 cyan
    (255,0,0),     # 8 red
    (255,128,0),   # 9 dark orange
    (255,165,0),   # A orange
    (255,255,0),   # B yellow
    (255,0,255),   # C magenta
    (255,128,255), # D pink
    (255,200,200), # E light pink
    (255,255,255), # F white
]

# === LOAD IMAGE ===
img = Image.open(IMAGE_PATH).convert("RGBA")
w, h = img.size

# calculate padding: center image to padded width
pad_left = (PADDED_WIDTH - w) // 2
pad_right = PADDED_WIDTH - w - pad_left

def closest_palette_index(r,g,b):
    best_idx = 0
    best_dist = 1e9
    for i,(pr,pg,pb) in enumerate(PALETTE):
        d = (r-pr)**2 + (g-pg)**2 + (b-pb)**2
        if d < best_dist:
            best_dist = d
            best_idx = i
    return best_idx

design = []
mask = []

longs_per_row = PADDED_WIDTH // 16

for y in range(h):
    row = []

    # left transparent padding
    for _ in range(pad_left):
        row.append(0xF)

    # actual sprite pixels
    for x in range(w):
        r,g,b,a = img.getpixel((x,y))
        if a == 0 or (r,g,b) == TRANSPARENT_COLOR:
            row.append(0xF)
        else:
            row.append(closest_palette_index(r,g,b))

    # right transparent padding
    for _ in range(pad_right):
        row.append(0xF)

    # pack into long longs
    for chunk in range(longs_per_row):
        start = chunk * 16
        nibs = row[start:start+16]

        # design 64-bit
        ll = 0
        for n in nibs:
            ll = (ll << 4) | n
        design.append(ll)

        # mask 1 bit per pixel (1 = transparent, 0 = draw)
        m = 0
        for n in nibs:
            m = (m << 1) | (1 if n == 0xF else 0)
        mask.append(m)

print(f"const int PIPE_W = {PADDED_WIDTH};")
print(f"const int PIPE_H = {h};\n")

print("const long long pipe_sprite[] = {")
for ll in design:
    print(f"  0x{ll:016X},")
print("};\n")

print("const long long pipe_mask[] = {")
for m in mask:
    print(f"  0x{m:016X},")
print("};\n")

print("// Paste PIPE_W, PIPE_H, pipe_sprite[], pipe_mask[] into your C file.")
