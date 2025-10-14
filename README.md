# CONTROL PID

## Que es

#### PID significa Proporcional , Integral y Derivativo . Cada uno de estos términos se refiere a una parte específica o función del bucle. Los bucles PID comparan el punto objetivo con la ubicación real y corrigen cualquier error para mantener el rumbo. Este proceso se repite una y otra vez para mantener resultados consistentes. 
#### 1. Control proporcional (P), que es proporcional al error. Cuanto mayor sea el error, mayor será la salida del control.
#### 2. Control integral (I), proporcional al error acumulado a lo largo del tiempo. Ayuda a eliminar errores de estado estacionario.
#### 3. Control derivativo (D), proporcional a la tasa de variación del error. Ayuda a amortiguar la respuesta del control y a evitar sobreimpulsos.

#### La salida de control es la suma de estos tres términos de control, que se combinan y ajustan para lograr el rendimiento de control deseado. El objetivo es mantener la variable del proceso lo más cerca posible del punto de ajuste, minimizando al mismo tiempo el sobreimpulso, la oscilación y otros comportamientos indeseables.

#### PID BASICO:

#### Ejemplo de Código C++

```C++
MiniPID pid=MiniPID(1,0,0);
//se colocan configuracions adicionales de PID

while(true){
  //get some sort of sensor value
  //set some sort of target value
  double output=pid.getOutput(sensor,target);
  //do something with the output
  delay(50);
}
```

### Implementacion con sensor de inercia


###### Fuentes usadas:
###### https://www.vexforum.com/t/implementing-the-inertial-sensor-to-pid-loop/92811
###### https://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
###### https://www.vexforum.com/t/introducing-jar-template-a-vexcode-pro-library-featuring-custom-pid-and-odometry/111793
