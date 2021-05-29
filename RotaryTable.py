from math import sin, pi

class rotary_table_2d:
    def __init__(self, DT, CurPos=(0, 0), max_rate=(720, 1200), max_accel=(100, 100)):
        self.T = 0
        self.DT = DT
        self.cur_pos = [CurPos[0], CurPos[1]]
        self.rate = [0, 0]
        self.accel = 0
        self.max_rate = max_rate
        self.max_accel = max_accel
        self.pos_history = [[CurPos[0]], [CurPos[1]]]
        self.rate_history = [[0], [0]]
        self.accel_history = [[0], [0]]
        self.time_history = [0]

    def POS(self, AxisNumber, Position, Rate=100):

        if (AxisNumber == 1 and (Rate < 0 or Rate > self.max_rate[0])):
            raise ValueError("Недопустимое значение скорости")
        if (AxisNumber == 2 and (Rate < 0 or Rate > self.max_rate[1])):
            raise ValueError("Недопустимое значение скорости")
        if AxisNumber not in (1, 2):
            raise ValueError("Неправильный номер оси вращения")

        self.rate[AxisNumber - 1] = Rate

        while (abs(Position - self.cur_pos[AxisNumber - 1]) > 1e-2):
            self.cur_pos[AxisNumber - 1] += Rate * self.DT
            self.check_pos()
            self.pos_history[0] += [self.cur_pos[0]]
            self.pos_history[1] += [self.cur_pos[1]]
            self.rate_history[0] += [self.rate[0]]
            self.rate_history[1] += [self.rate[1]]
            self.accel_history[0] += [0]
            self.accel_history[1] += [0]
            self.T += self.DT
            self.time_history += [self.T]

        self.rate = [0, 0]

    def PRS(self, AxisNumber, Position, Rate=10):

        if (AxisNumber == 1 and (Rate < 0 or Rate > self.max_rate[0])):
            raise ValueError("Недопустимое значение скорости")
        if (AxisNumber == 2 and (Rate < 0 or Rate > self.max_rate[1])):
            raise ValueError("Недопустимое значение скорости")
        if AxisNumber not in (1, 2):
            raise ValueError("Неправильный номер оси вращения")

        self.rate[AxisNumber - 1] = Rate

        while (abs(Position - self.cur_pos[AxisNumber - 1]) > 1e-2):
            if Position > self.cur_pos[AxisNumber - 1]:
                self.cur_pos[AxisNumber - 1] += Rate * self.DT
            else:
                self.cur_pos[AxisNumber - 1] -= Rate * self.DT
            self.check_pos()
            self.pos_history[0] += [self.cur_pos[0]]
            self.pos_history[1] += [self.cur_pos[1]]
            self.rate_history[0] += [self.rate[0]]
            self.rate_history[1] += [self.rate[1]]
            self.accel_history[0] += [0]
            self.accel_history[1] += [0]
            self.T += self.DT
            self.time_history += [self.T]

        self.rate = [0, 0]

    def PRV(self, AxisNumber, Position, Rate=10, Acceleration=10):

        self.rate[AxisNumber - 1] = 0

        if (AxisNumber == 1 and (Rate < 0 or Rate > self.max_rate[0])):
            raise ValueError("Недопустимое значение скорости")
        if (AxisNumber == 2 and (Rate < 0 or Rate > self.max_rate[1])):
            raise ValueError("Недопустимое значение скорости")
        if (AxisNumber == 1 and (Acceleration < 0 or Acceleration > self.max_accel[0])):
            raise ValueError("Недопустимое значение ускорения")
        if (AxisNumber == 2 and (Acceleration < 0 or Acceleration > self.max_accel[1])):
            raise ValueError("Недопустимое значение ускорения")
        if AxisNumber not in (1, 2):
            raise ValueError("Неправильный номер оси вращения")

        while (abs(Position - self.cur_pos[AxisNumber - 1]) > 1e-2):
            if self.rate[AxisNumber - 1] < Rate:
                self.rate[AxisNumber - 1] += Acceleration * self.DT
            if Position > self.cur_pos[AxisNumber - 1]:
                self.cur_pos[AxisNumber -
                             1] += self.rate[AxisNumber - 1] * self.DT
            else:
                self.cur_pos[AxisNumber -
                             1] -= self.rate[AxisNumber - 1] * self.DT
            self.check_pos()
            self.pos_history[0] += [self.cur_pos[0]]
            self.pos_history[1] += [self.cur_pos[1]]
            self.rate_history[0] += [self.rate[0]]
            self.rate_history[1] += [self.rate[1]]
            if AxisNumber == 1:
                self.accel_history[0] += [Acceleration]
                self.accel_history[1] += [0]
            else:
                self.accel_history[0] += [0]
                self.accel_history[1] += [Acceleration]
            self.T += self.DT
            self.time_history += [self.T]

        self.rate = [0, 0]

    def RAT(self, AxisNumber, TMODEL=60, Rate=10, Acceleration=10):

        self.rate[AxisNumber - 1] = 0

        if (AxisNumber == 1 and (Rate < 0 or Rate > self.max_rate[0])):
            raise ValueError("Недопустимое значение скорости")
        if (AxisNumber == 2 and (Rate < 0 or Rate > self.max_rate[1])):
            raise ValueError("Недопустимое значение скорости")
        if (AxisNumber == 1 and (Acceleration < 0 or Acceleration > self.max_accel[0])):
            raise ValueError("Недопустимое значение ускорения")
        if (AxisNumber == 2 and (Acceleration < 0 or Acceleration > self.max_accel[1])):
            raise ValueError("Недопустимое значение ускорения")
        if AxisNumber not in (1, 2):
            raise ValueError("Неправильный номер оси вращения")

        T_start = self.T
        while (self.T - T_start < TMODEL):
            if self.rate[AxisNumber - 1] < Rate:
                self.rate[AxisNumber - 1] += Acceleration * self.DT
            self.cur_pos[AxisNumber - 1] += self.rate[AxisNumber - 1] * self.DT
            self.check_pos()
            self.pos_history[0] += [self.cur_pos[0]]
            self.pos_history[1] += [self.cur_pos[1]]
            self.rate_history[0] += [self.rate[0]]
            self.rate_history[1] += [self.rate[1]]
            if AxisNumber == 1:
                self.accel_history[0] += [Acceleration]
                self.accel_history[1] += [0]
            else:
                self.accel_history[0] += [0]
                self.accel_history[1] += [Acceleration]
            self.T += self.DT
            self.time_history += [self.T]

        self.rate = [0, 0]

    def SIN(self, AxisNumber, TMODEL, Amp, Freq, Phase):

        if AxisNumber not in (1, 2):
            raise ValueError("Неправильный номер оси вращения")

        T_start = self.T

        while (self.T - T_start < TMODEL):
            self.rate[AxisNumber - 1] = 2*pi*Freq*Amp *                 sin(2*pi*Freq*(self.T - T_start) + Phase/57.3)
            self.cur_pos[AxisNumber - 1] += self.rate[AxisNumber - 1] * self.DT
            self.check_pos()
            self.pos_history[0] += [self.cur_pos[0]]
            self.pos_history[1] += [self.cur_pos[1]]
            self.rate_history[0] += [self.rate[0]]
            self.rate_history[1] += [self.rate[1]]
            if AxisNumber == 1:
                self.accel_history[0] += [2*pi*Freq*2*pi*Freq*Amp *
                                          sin(2*pi*Freq*(self.T - T_start) + Phase/57.3 + pi/2)]
                self.accel_history[1] += [0]
            else:
                self.accel_history[1] += [2*pi*Freq*2*pi*Freq*Amp *
                                          sin(2*pi*Freq*(self.T - T_start) + Phase/57.3 + pi/2)]
                self.accel_history[0] += [0]
            self.T += self.DT
            self.time_history += [self.T]

        self.rate = [0, 0]

    def check_pos(self):
        self.cur_pos[0] = self.cur_pos[0] % 360
        self.cur_pos[1] = self.cur_pos[1] % 360