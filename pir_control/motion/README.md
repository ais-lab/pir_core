## Motion script manual (.mts)

1. __acceleration__
    - *parameter* : acceleration value(mm/s^2), target speed(mm/s)
    - *ex* : acceleration 700 480, acceleration -500 -230

1. __forward__
    - *parameter* : speed(mm/s), distance(mm)
    - *ex* : forward 480 1000, forward -200 3000

1. __rotation__
    - *parameter* : omega(rad/s), angle(degree)
    - *ex* : rotation 1.8 340, rotation -1.0 90

1. __turning__
    - *parameter* : speed(mm/s), radius(mm), distance(mm), 'left'or'right'
    - *ex* : turning 400 1000 3140 left, turning -300 500 1000 right

1. __pause__
    - *parameter* : time(sec)
    - *ex* : pause 1000

1. __stop__
    - *parameter* :
    - *ex* : stop

1. __slowstop__
    - *parameter* : deceleration value(mm/s^2)
    - *ex* : slowstop 500

1. __detect__
    - *parameter* : distance(m), file name
    - *ex* : detect 2 sample

1. __e__
    - *parameter* : file name
    - *ex* : e sample

1. __end__
    - *parameter* :
    - *ex* : end
