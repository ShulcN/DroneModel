<!-- # Содержание -->




<!-- # Мои размышления




## Моё понимание подходов

### Линеаризация по обратной связи

С помощью такой линеаризации мы удаляем нелинейности системы посредством
 -->



# Статьи


## 1 PID+Backstepping

название: Trajectory tracking control design with command-ﬁltered compensation for
quadrotor

2015 год 


Система управления: Используется PID с бэкстеппингом. Два контура управления: внешний и внутренний: по координатам XYZ и для вращения.

Уравнения модели похожи. Вывод более-менее понятный. Слабо, эффективность сомнительная.

[link](https://www.researchgate.net/publication/224198367)


## 2 LQR+Backestepping 

Название: Saturated Trajectory Tracking Controller in the Body-Frame
for Quadrotors

2024 год

Система управления: LQR на внешний контур + Backstepping на внутренний

Уравнения хорошие, текст понятный, результаты вроде бы неплохие

[link](https://doi.org/10.3390/drones8040163)


## 3 Robust backstepping sliding mode

Название: Robust Backstepping Sliding Mode
Control for a Quadrotor Trajectory
Tracking Application

2016 год

Система управления: снова два контура, как в предыдущих статьях. Для внутреннего контура - backstepping sliding mode controller;
для внешнего контура: integral sliding mode control.

Моделирование интересное, при сильном ветре всё равно робастность хорошая.
Написано тяжело... Но выглядит интересно

[link](https://www.researchgate.net/publication/338202010_Robust_Backstepping_Sliding_Mode_Control_for_a_Quadrotor_Trajectory_Tracking_Application)

## 5 LQR+feedback


Название: Inner-outer feedback linearization for quadrotor control:
two-step design and validation

2022 год 

Система управления: два контура, но немного необычно - 
внутренний контур угол крена (ϕ), угол тангажа (θ), угол рыскания (ψ) и высотой (z);
внешний контур - положение квадрокоптера в горизонтальной плоскости (x, y).

Во внутреннем и внешнем контуре: линеаризация по обратной связи + LQR(но вроде бы не совсем обычный с интегральной частью);
В целом система асимптотически устойчива в достаточно большом диапазоне:


Хорошо написано, не выглядит очень сложно. 
Результаты экспериментов не идеальны, но и траектория там сложная.
А в целом у них там сравнительная таблица с некоторыми регуляторами из других статей, у них показатели лучше.


[link](https://www.researchgate.net/publication/361626973_Inner-outer_feedback_linearization_for_quadrotor_control_two-step_design_and_validation)

# 6 MPC GP

Название: Gaussian Process Model Predictive Control
of an Unmanned Quadrotor

2016 год

Мне скорее всего не подходит так, как там основано на GP модели,
которая я так понял получается эмпирически. Тоже интересно, но не то

[link](https://www.researchgate.net/publication/315981809_Gaussian_Process_Model_Predictive_Control_of_An_Unmanned_Quadrotor)


# 7 qLPV MPC

Название: A qLPV-MPC Control Strategy for Trajectory Tracking
of Quadrotors

2023 год

Система управления: MPC+qLPV. То есть там используется вроде как 
какое-то частично линейное представление и на основе него делается MPC.
Таким образом снижается вычислительная нагрузка.

Может быть написано и нормально, но нужно почитать дополнительно про LPV.
Идея интересная очень. Результаты у них хорошие, но там и нет никаких сопротивлений. Не уверен, что хорошая робастность у этого контроллера, может быть если его дополнить, то будет славно

[link](https://www.researchgate.net/publication/372485762_A_qLPV-MPC_Control_Strategy_for_Trajectory_Tracking_of_Quadrotors)


# 8 MPC

Название: A constrained error-based MPC for path following of
quadrotor with stability analysis

2018 год

Система управления: два контура, оба управляются с помощью MPC. Насколько я понял там используется дискретная линейная модель для MPC.


Статья тоже интересная. Вроде бы более-менее подробно расписано. Но нужно конечно разобраться, как там проводили дискретизация, там как-то очень просто описано, как будто я что-то не понял.


[link](https://www.researchgate.net/publication/332426391_A_constrained_error-based_MPC_for_path_following_of_quadrotor_with_stability_analysis)



# 9 feedback linearization


Название: Nonlinear Control of a Quadrotor Micro-UAV using
Feedback-Linearization

2009 год

Система управления: два контура, feedback линеаризация + П и ПД регуляторы.

Фишка в том, что тут идет управление по скорости. Статья не очень, описание не подробное.

[link](https://www.researchgate.net/publication/224459823_Nonlinear_control_of_a_quadrotor_micro-UAV_using_feedback-linearization)


# 10 RANFTSMC

Название: Robust adaptive nonsingular fast terminal sliding-mode tracking control for
an uncertain quadrotor UAV subjected to disturbances

2020 год

Система управления: два контура, на каждом из которых 
применяется предложенный алгоритм robust ANFTSMC.


Ну судя по тому, что показывают товарищи в своей статье, 
получилось очень круто. Расчеты сложные, нужно 


[link](https://www.researchgate.net/publication/337010656_Robust_adaptive_nonsingular_fast_terminal_sliding-mode_tracking_control_for_an_uncertain_quadrotor_UAV_subjected_to_disturbances)


# 11 adaptive backstepping

Название: Adaptive trajectory tracking control design with command filtered compensation for a quadrotor

2013 год

Система управления: регулятор backstepping + адаптивные законы для оценки внешних возмущений


Тоже выглядит интересно, вроде бы более-менее понятно. По сравнению с RANFTSMC это выглядит понятно


[link](https://www.researchgate.net/publication/252034521_Adaptive_trajectory_tracking_control_design_with_command_filtered_compensation_for_a_quadrotor)

# 12 NMPC vs Feedback controller

Название: A Comparative Study between NMPC and Baseline Feedback Controllers for UAV Trajectory Tracking

2023 год

Система управления:
NMPC и линеаризация по обратной связи. Но тут используются два варианта с линеаризацией: 
линеаризация только по кинематике и линеаризация по кинематике + по динамике.


Мало формул, статья больше именно про сравнение, чем про
то, как это строить


[link](https://www.researchgate.net/publication/368677043_A_Comparative_Study_between_NMPC_and_Baseline_Feedback_Controllers_for_UAV_Trajectory_Tracking)



# 13 MPC

Название: Closed-Loop Model Identification and
MPC-based Navigation of Quadcopters: A
Case Study of Parrot Bebop 2

2024 год

Система управления: MPC по дискретизированной системе

[link](https://arxiv.org/pdf/2404.07267)

# 14 Feedback linearization + EMC

Название: Feedback Linearization for Quadrotors UAV

2019 год

Система управления: feedback linearization + EMC

Интересная статья, в том плане, что тут тема ровно про линеаризцаию по обратной связи, но как-то непривычно написано, нет моделирования, да и в целом система управления не предназначена именно для слежения за траекторией

[link](https://arxiv.org/pdf/1906.04263)




# Дополнительные штуки

## Статья на матлаб

Nonlinear MPC for quadrotor [link](https://www.mathworks.com/help/mpc/ug/control-of-quadrotor-using-nonlinear-model-predictive-control.html)



