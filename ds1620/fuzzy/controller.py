# *******************************************************************************************************
# Селетков И.П. 2018 0417.                                                                              *
# Класс для управления температурой объёктов с помощью нечёткой логики.                                 *
# *******************************************************************************************************

from fuzzy.vector import CFuzzyVector

class CFuzzyController:
    # ***************************************************************************************************
    # Конструктор.                                                                                      *
    # @param target - целевое значение температуры.                                                     *
    # ***************************************************************************************************
    def __init__(self, target=0):
        self.target = target

        # Очередь из значений температур для хранения небольшой истории.
        self.__t1 = target
        self.__t2 = target
        self.__t3 = target

    # ***************************************************************************************************
    # Температура очень низкая.                                                                         *
    # @param value - точное значение темперартуры.                                                      *
    # @return степень истинности высказывания.                                                          *
    # ***************************************************************************************************
    def __very_low(self, value):
        value = value-self.target
        if value <= -20:
            return 1
        elif value >= -10:
            return 0
        else:
            return -(value+10)/10

    # ***************************************************************************************************
    # Температура низкая.                                                                               *
    # @param value - точное значение темперартуры.                                                      *
    # @return степень истинности высказывания.                                                          *
    # ***************************************************************************************************
    def __low(self, value):
        value = value - self.target
        if value <= -20:
            return 0
        elif value >= -5:
            return 0
        elif -20 < value <= -10:
            return (value+20)/10
        else:
            return -(value + 5) / 5

    # ***************************************************************************************************
    # Температура чуть-чуть низкая.                                                                     *
    # @param value - точное значение темперартуры.                                                      *
    # @return степень истинности высказывания.                                                          *
    # ***************************************************************************************************
    def __slightly_low(self, value):
        value = value - self.target
        if value <= -10:
            return 0
        elif value >= 0:
            return 0
        elif -10 < value <= -5:
            return (value + 10) / 5
        else:
            return -(value + 0) / 5

    # ***************************************************************************************************
    # Температура равна уставке.                                                                        *
    # @param value - точное значение темперартуры.                                                      *
    # @return степень истинности высказывания.                                                          *
    # ***************************************************************************************************
    def __equal_target(self, value):
        value = value - self.target
        if value <= -5:
            return 0
        elif value >= 5:
            return 0
        elif -5 < value <= 0:
            return (value + 5) / 5
        else:
            return -(value - 5) / 5

    # ***************************************************************************************************
    # Температура чуть-чуть высокая.                                                                    *
    # @param value - точное значение темперартуры.                                                      *
    # @return степень истинности высказывания.                                                          *
    # ***************************************************************************************************
    def __slightly_high(self, value):
        value = value - self.target
        if value <= 0:
            return 0
        elif value >= 10:
            return 0
        elif 0 < value <= 5:
            return (value + 0) / 5
        else:
            return -(value - 10) / 5

    # ***************************************************************************************************
    # Температура высокая.                                                                              *
    # @param value - точное значение темперартуры.                                                      *
    # @return степень истинности высказывания.                                                          *
    # ***************************************************************************************************
    def __high(self, value):
        value = value - self.target
        if value <= 5:
            return 0
        elif value >= 20:
            return 0
        elif 5 < value <= 10:
            return (value - 5) / 5
        else:
            return -(value - 20) / 10

    # ***************************************************************************************************
    # Температура очень высокая.                                                                        *
    # @param value - точное значение темперартуры.                                                      *
    # @return степень истинности высказывания.                                                          *
    # ***************************************************************************************************
    def __very_high(self, value):
        value = value - self.target
        if value <= 10:
            return 0
        elif value >= 20:
            return 1
        else:
            return -(value - 20) / 10

    # ***************************************************************************************************
    # Расчитывает и возвращает доли мощностей нагревателя и охладителя.                                 *
    # @param value - точное значение темперартуры.                                                      *
    # @return (доля мощности нагревателя, доля мощности охладителя).                                    *
    # ***************************************************************************************************
    def get(self, value):
        # Актуализируем содержимое памяти
        self.__t3 = self.__t2
        self.__t2 = self.__t1
        self.__t1 = value

        # Если температура очень низкая
        pvl = CFuzzyVector(self.__very_low(value))
        pml = CFuzzyVector(self.__low(value))
        psl = CFuzzyVector(self.__slightly_low(value))
        peq = CFuzzyVector(self.__equal_target(value))
        psh = CFuzzyVector(self.__slightly_high(value))
        pmh = CFuzzyVector(self.__high(value))
        pvh = CFuzzyVector(self.__very_high(value))

        peq2 = CFuzzyVector(self.__equal_target(self.__t2))
        peq3 = CFuzzyVector(self.__equal_target(self.__t3))

        # Три раза подряд температура равна уставке, немного подогреваем,
        # чтобы выровнять смоохлаждение.
        peq1 = peq.c(peq2.c(peq3))

        heater = pvl.x*100 + pml.x*50+psl.x*20+peq.x*0+psh.x*0+pmh.x*0+pvh.x*0+peq1.x*7
        if heater>0:
            heater /= pvl.x + pml.x+psl.x+peq.x+psh.x+pmh.x+pvh.x+peq1.x

        cooler = 0

        return heater, cooler
