from time import sleep
import SerialInterface
import numpy as np
import sys
import argparse

"""Notes:

9/3/2023 Eric Weeks

Vreg 1 = TES1 = INA 3
Vreg 2 = TES2 = INA 4
Vreg 3 = TES3 = INA 5
Vreg 4 = TES4 = INA 6
Vreg 5 = LNA1 D = INA 1
Vreg 6 = LNA1 G = INA 7
Vreg 7 = LNA2 D = INA 2
Vreg 8 = LNA2 G = INA 8

Python Interface: (Using 1 line commands, not a GUI)

[*]vTES Ch V(uV) //Adjusts POT wiper and then reads INA values until I x 400uOhm = V(uV) on Ch (Vreg 1-4)
[*]iTES Ch I(mA) //Adjusts POT wiper and then reads INA values until I = I(mA) on Ch (Vreg 1-4)
[*]vLNA Ch D V(V) //Adjusts POT wiper and then reads INA values until V(bus) - V(shunt) = V(V) on Ch (Vreg 5 and 7)
[*]iLNA Ch I(mA) //Adjusts POT wiper and then reads INA values until I = I(mA) for Ch
[*]vLNA Ch G V(V) //Adjusts POT wiper and then reads INA values until V(bus) = V(V) on Ch (Vreg 6 and 8)

[*]getiTES Ch
[*]getvTES Ch
[ ]get all iv //Reads INA values for all Ch. Labels should be displayed as TES1, TES2,... LNA1 D, LNA2 D...
			 INA7 and INA8 are not used to measure current so only report LNA VG (VG voltages need a sign change +-).
"""

VTES_TOLERANCE = 0.05
ITES_TOLERANCE = 0.01
VLNA_D_TOLERANCE = 0.01
VLNA_G_TOLERANCE = 0.01
ILNA_TOLERANCE = 0.01

INA219CHANMAP = [0, 3, 4, 5, 6, 1, 2]
NAMES = [
    "INA TES1",
    "INA TES2",
    "INA TES3",
    "INA TES4",
    "LNA D Bias 1",
    "LNA D Bias 2",
]


class LNA:
    def __init__(self, SerialPort) -> None:
        self.si = SerialInterface.SerialController(SerialPort)
        self.si.open()
        self.lna_wiper_g = [0, 0]

    def end(self):
        self.si.close()

    def vTES(self, ch, uv):
        potvalue = self.si.get_wiper(1, ch - 1)

        MAXITER = 275
        i = 0
        while 1:
            if i > 512:
                print(f"|ERROR| loop exceeded {MAXITER}")
                break

            v_measured = self.get_vTES(ch)
            diff = abs(v_measured - uv) / uv
            print(f"vmeasured= {v_measured}; diff= {diff}")
            if diff < VTES_TOLERANCE:
                break

            if v_measured < uv:
                if potvalue == 255:
                    print("|WARN| Pot is at max setting!")
                    break
                potvalue += 1
                self.si.set_wiper(1, ch - 1, potvalue)

            elif v_measured > uv:
                if potvalue == 0:
                    print("|WARN| Pot is at min setting")
                    break
                potvalue -= 1
                self.si.set_wiper(1, ch - 1, potvalue)
            i += 1

    def iTES(self, ch, imA):
        potvalue = self.si.get_wiper(1, ch - 1)

        MAXITER = 275
        i = 0
        while 1:
            if i > 512:
                print(f"|ERROR| loop exceeded {MAXITER}")
                break

            i_measured = self.get_iTES(ch)
            diff = abs(i_measured - imA) / imA
            print(f"i-measured= {i_measured}; diff= {diff}")
            if diff < ITES_TOLERANCE:
                break

            if i_measured < imA:
                if potvalue == 255:
                    print("|WARN| Pot is at max setting!")
                    break
                potvalue += 1
                self.si.set_wiper(1, ch - 1, potvalue)

            elif i_measured > imA:
                if potvalue == 0:
                    print("|WARN| Pot is at min setting")
                    break
                potvalue -= 1
                self.si.set_wiper(1, ch - 1, potvalue)
            i += 1
        pass

    def vLNA_D(self, lnaBias, V):
        # FIXME ALWAYS STARTS AT 0
        ch = wiper = 0
        if lnaBias == 1:
            wiper = 1 - 1
        else:
            wiper = 3 - 1

        potvalue = self.si.get_wiper(2, wiper)

        MAXITER = 275
        i = 0
        while 1:
            if i > 512:
                print(f"|ERROR| loop exceeded {MAXITER}")
                break
            vals = np.zeros(10)
            for i, v in enumerate(vals):
                bus_V, shunt_mV, curr_mA = self.si.get_bsi(lnaBias - 1)
                vals[i] = bus_V
            bus_V = np.average(vals)

            Vmeasured = bus_V + 0.07

            diff = abs(Vmeasured - V) / V
            print(f"vLNA_D V measured= {Vmeasured}; diff= {diff}")
            if diff < VLNA_D_TOLERANCE:
                break

            if Vmeasured < V:
                if potvalue == 255:
                    print("|WARN| Pot is at max setting!")
                    break
                potvalue += 1
                self.si.set_wiper(2, wiper, potvalue)

            elif Vmeasured > V:
                if potvalue == 0:
                    print("|WARN| Pot is at min setting")
                    break
                potvalue -= 1
                self.si.set_wiper(2, wiper, potvalue)
            i += 1

    def iLNA(self, lnaBias, setCurrent):
        # FIXME ALWAYS STARTS AT 0
        ch = wiper = 0
        if lnaBias == 1:
            wiper = 1 - 1
        else:
            wiper = 3 - 1

        potvalue = self.si.get_wiper(2, wiper)

        MAXITER = 275
        i = 0
        while 1:
            if i > 512:
                print(f"|ERROR| loop exceeded {MAXITER}")
                break
            vals = np.zeros(10)
            for i, v in enumerate(vals):
                bus_V, shunt_mV, curr_mA = self.si.get_bsi(lnaBias - 1)
                vals[i] = curr_mA
            curr_mA = np.average(vals) / 100

            diff = abs(curr_mA - setCurrent) / setCurrent
            print(f"iLNA_D current measured= {curr_mA}; diff= {diff}")
            if diff < ILNA_TOLERANCE:
                break

            if curr_mA < setCurrent:
                if potvalue == 255:
                    print("|WARN| Pot is at max setting!")
                    break
                potvalue += 1
                self.si.set_wiper(2, wiper, potvalue)

            elif curr_mA > setCurrent:
                if potvalue == 0:
                    print("|WARN| Pot is at min setting")
                    break
                potvalue -= 1
                self.si.set_wiper(2, wiper, potvalue)
            i += 1

    def vLNA_G(self, lna, voltage):
        wiper = int(round(voltage / 0.000784))
        if wiper > 255:
            wiper = 255
        if wiper < 0:
            wiper = 0
        self.si.set_wiper(2, (lna * 2) - 1, wiper)

        self.lna_wiper_g[lna - 1] = wiper

    def get_vTES(self, ch):
        vals = np.zeros(10)
        for i, v in enumerate(vals):
            b, s, curr = self.si.get_bsi(INA219CHANMAP[ch] - 1)
            vals[i] = curr
        VuV = (np.average(vals) * 1e-3) * 0.4e-3 * 1e6 * 1e-2
        return VuV

    def get_iTES(self, ch):
        vals = np.zeros(10)
        for i, v in enumerate(vals):
            b, s, curr = self.si.get_bsi(INA219CHANMAP[ch] - 1)
            vals[i] = curr
        return np.average(vals) * 1e-2

    def getAllIV(self):
        busVs = np.zeros(6)
        shuntMvs = np.zeros(6)
        currents = np.zeros(6)
        print(
            "**************************************** INA READINGS ****************************************"
        )
        print(
            "Channel \t\t INA219 \t Bus Voltage (V) \t Shunt Voltage (mV) \t\t Current (mA)"
        )
        for i in range(6):
            b, s, curr = self.si.get_bsi(INA219CHANMAP[i + 1] - 1)
            busVs[i] = b
            shuntMvs[i] = s
            currents[i] = curr / 100
            print(
                f"{NAMES[i]}    \t\t       {INA219CHANMAP[i+1]}       \t       {b}        \t      {s}      \t        {curr/100}              "
            )
        print(f"\nVG LNA1 = {self.lna_wiper_g[0]*.000784}")
        print(f"\nVG LNA2 = {self.lna_wiper_g[1]*.000784}")
        print("\n\n")


def test():
    l = LNA("COM10")
    l.si.set_allgpio(0b111111111)
    # l.si.set_gpio(0, 1)
    # l.si.set_gpio(1, 1)
    # l.si.set_gpio(5, 1)
    l.si.set_wiper(1, 0, 255)
    l.si.set_wiper(1, 1, 255)
    l.si.set_wiper(1, 2, 255)
    l.si.set_wiper(1, 3, 255)
    l.si.set_wiper(2, 2 - 1, 255)
    l.si.set_wiper(1, 2, 255)
    # l.vTES(1, 1.00)
    # l.iTES(1, 1.3)
    # l.vLNA_D(1, 3.0)
    # l.iLNA(1, 20)
    l.vLNA_G(1, 0.000784)
    l.vLNA_G(2, 0.000784 * 2)
    l.getAllIV()
    l.end()


if __name__ == "__main__":
    test()
