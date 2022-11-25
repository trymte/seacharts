from simcharts.enc import ENC

def main():
    size = 900.0, 506.20
    # size = 20000, 12062
    origin = 569042.27, 7034900.20
    center = origin[0] + size[0]//2, origin[1] + size[1]//2
    enc = ENC(border=True)

    enc.add_ownship(center[0], center[1], 0.1, hull_scale=0.2)
    
    enc.show_display()

if __name__ == '__main__':
    main()