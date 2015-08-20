# spsvi
  Система пространственной стабилизации видеоизображений (spsvi) представляет собой программно-аппаратный комплекс стабилизирующий платформу с камерой в пространстве. 

  В этом репозитории представлен проект для среды IAR Embedded Workbench for ARM, в котором приведен код для микропроцессора STM32L. В main.cpp реализованы алгоритмы считывания показаний инерциальных датчиков (акселерометр, гироскоп, магнитометр) с платы GY-80 по шине I2C, фильтрация полученных данных и преобразование их в углы наклона камеры с использованием альфа-бета фильтра, алгоритм ПИД для управления положением камеры в пространстве при помощи ШИМ, который реализован с помощью одного таймера и трех каналов захвата/сравнения.
  Так же присутствуют готовые Matlab-файлы, в которых реализованы алгоритм захвата видеоизображений с веб-камеры, запись их в avi - файл и алгоритм распознавания угла наклона на каждом кадре готового видеоизображения относительно последнего, с использованиям SURF-детектора особых точек и дескрипторов.  Использовался при анализе работы устройства.
  Управлялка spsvi - программа, написанная на Delphi XE3. Позволяет прямо "на ходу" во время работы менять значения коэффициентов ПИД-регулятора для данного комплекса при помощи графического интерфейса. Для компиляции требуется сторонний компонент - TComPort. Так же программа постоянно считывает и отображает значения положения камеры в пространстве по трем осям (крен, тангаж, рыскание), ошибку регулирования (ПИД) и  текущее положение сервоприводов.

Автор - Лазурин Е. В. 
По всем вопросам как этим пользоваться и как оно работает, а так же Ваши замечания пишите на zhenya1993lzn@yandex.ru
2015 год
