# Robot
Debido a que se ha realizado un desarrollo guiado por test del proyecto, ha sido necesario realizar
una serie de cambios en la manera en la que se definen las funciones de la especificaci

## Cambios realizados respecto a la especificación
### Funcionalidades cambiadas

`read_speed` y `set_speed` se han separado en diferentes clases:
* `read_speed` esta en la clase `Odometry()` en el fichero `robotics/robot/odometry.py` 
* `set_speed` esta en la clase `Robot` del fichero `robotics/robot/robot.py`

### Odometry

* `update_odometry` es el método `Odometry.update()` y 
* `read_odometry` se ha sustituido por el método `Odometry.location()` que devuelve la localización absoluta del robot.


## Ejecución
Para ejecutar: 
* `python robotics/__main__.py`
