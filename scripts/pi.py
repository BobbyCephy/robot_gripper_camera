def pi(m, d, t, r=2):
    Km = 1/d
    Tm = m/d
    tm = t

    if r <= 1:
        if not r:
            Kc = 0.556
            Ti = 3.7

        else:
            Kc = 0.952
            Ti = 4

        Kc /= Km * (Tm + tm)
        Ti *= (Tm + tm)

    else:
        Kc = 1.477/Km*Tm/tm**2 * (1 + (Tm/tm)**(0.65))**(-2)
        Ti = 3.33*tm * (1 + (Tm/tm)**(0.65))

    p = Kc
    i = Kc / Ti
    return map(float, (p, i))
    
if __name__ == "__main__":
    print(*pi(1.03804, 130/0.15, 1/50))