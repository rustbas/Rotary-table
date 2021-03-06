# Rotary-table

## Описание

Данная программа реализует класс поворотного стола. В файле Jupyter Notebook хранится реализация класса, а также пример определения объекта и графики, полученные в процессе моделирования. В файле RotaryTable.py содержится только реализация класса.

## Аргументы конструктора

- DT — шаг дискретности (лучше делать как можно меньше)
- cur_pos — кортеж с начальной позицией
- max_rate — кортеж максимальной скорости
- max_accel — кортеж максимального ускорения

## Методы класса

- POS(AxisNumber, Position, Rate) — позиционная управляющая команда. Достигается требуемая позиция всегда путем увеличения угла. Скорость меняется скачкообразно. Аргументы:
  - AxisNumber — номер оси (1 или 2)
  - Position — Требуемая позиция (в градусах)
  - Rate — скорость поворота, не больше максимальной

- PRS(AxisNumber, Position, Rate) - команда кратчайшего пути. Достигается требуемая позиция всегда кратчайшим путем. Скорость меняется скачкообразно. Аргументы:
  - AxisNumber — номер оси (1 или 2)
  - Position — Требуемая позиция (в градусах)
  - Rate — скорость поворота, не больше максимальной

- PRV(AxisNumber, Position, Rate, Acceleration) — команда кратчайшего пути с ускорением. Достигается требуемая позиция всегда кратчайшим путем. Скорость увеличивается линейно, а уменьшается скачкообразно. Аргументы:
  - AxisNumber — номер оси (1 или 2)
  - Position — Требуемая позиция (в градусах)
  - Rate — скорость поворота, не больше максимальной
  - Acceleration — угловое ускорение, не больше максимальной
  
- RAT(AxisNumber, TMODEL, Rate, Acceleration) — команда задания угловой скорости движения поворотному симулятору. Скорость увеличивается линейно, а уменьшается скачкообразно. Аргументы:
  - AxisNumber — номер оси (1 или 2)
  - TMODEL — время моделирования 
  - Position — Требуемая позиция (в градусах)
  - Rate — скорость поворота, не больше максимальной
  - Acceleration — угловое ускорение, не больше максимальной

- SIN(AxisNumber, TMODEL, Amp, Freq, Phase) — синусоидальная управляющая команда. Аргументы:
  - AxisNumber — номер оси (1 или 2)
  - TMODEL — время моделирования 
  - Amp — Амплитуда сигнала (градусы)
  - Freq — Частота (Гц)
  - Phase — Сдвиг по фазе (градусы)

- check_pos() — команда проверки принадлежности угла промежутку от 0 до 360 градусов

## Атрибуты класса

- time_history — список, хранящий все значения времени для построения графиков
- accel_history[0] — список, хранящий все значения ускорения первой оси для построения графиков
- accel_history[1] — список, хранящий все значения ускорения второй оси для построения графиков
- rate_history[0] — список, хранящий все значения скорости первой оси для построения графиков
- rate_history[1] — список, хранящий все значения скорости второй оси для построения графиков
- pos_history[0] — список, хранящий все значения угла на первой оси для построения графиков
- pos_history[1] — список, хранящий все значения угла на второй оси для построения графиков

# N.B.
Если нужно экспортировать значения углов, скоростей и т.п., то можно использовать атрибуты с пометкой "*\_history" 
