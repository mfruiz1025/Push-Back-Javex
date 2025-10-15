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
### Codigo reutilizable

```C++
// PID simple reutilizable
struct PID {
  double kp, ki, kd;
  double integral = 0.0;
  double lastError = 0.0;
  double outMin = -100.0;
  double outMax = 100.0;
  double integralMin = -1e6;
  double integralMax = 1e6;

  PID(double p=0,double i=0,double d=0) : kp(p), ki(i), kd(d) {}

  void setOutputLimits(double minv, double maxv) { outMin = minv; outMax = maxv; }
  void setIntegralLimits(double minv, double maxv) { integralMin = minv; integralMax = maxv; }

  void reset() { integral = 0.0; lastError = 0.0; }

double update(double error, double dt) {
    if(dt <= 0.0) dt = 0.02;
    integral += error * dt;
    // anti-windup integral clamp
    if(integral > integralMax) integral = integralMax;
    if(integral < integralMin) integral = integralMin;
    double derivative = (error - lastError) / dt;
    double out = kp * error + ki * integral + kd * derivative;
    // clamp output
    if(out > outMax) out = outMax;
    if(out < outMin) out = outMin;
    lastError = error;
    return out;
  }
};

```
## Ajustes que se necesitan:

Kp=0.6: fuerza principal para alcanzar distancia.
    * Aumentar si: avance muy lento.
    *Disminuir si: overshoot / oscila al parar.
* Ki=0.001: corrige error estacionario en distancia.
    * Aumentar si: siempre queda corto (error persistente).
    * Disminuir si: aparece oscilación lenta o integral crece mucho.
* Kd=0.02: amortigua llegada.
    * Aumentar si: rebota al llegar.
    * Disminuir si: amplifica ruido o frena demasiado.
* integral limits = [-500,500] → reduce windup

  
pidHeading(1.5, 0.0, 0.05)
* Kp=1.5: corrige desviaciones de rumbo
    * Aumentar si: se desvía mucho
    * Disminuir si: hace correcciones bruscas/oscila
* Ki=0.0: normalmente no se usa en heading
* Kd=0.05: suaviza correcciones
    * Aumentar si: overshoot en correcciones
    * Disminuir si: ruido amplificado
      
pidRotate(0.9, 0.0005, 0.04)
* Kp=0.9: gira con fuerza proporcional
    * Aumentar si: giro lento
    * Disminuir si: rebasa y oscila
* Ki=0.0005: corrige pequeño error final
    * Aumentar si: queda angulo persistente
    * Disminuir si: deriva u oscilación lenta
* Kd=0.04: amortigua giro
    * Aumentar si: rebotes al frenar
    *Disminuir si: respuesta muy lenta o ruido





###### Fuentes usadas:
###### https://www.vexforum.com/t/implementing-the-inertial-sensor-to-pid-loop/92811
###### https://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
###### https://www.vexforum.com/t/introducing-jar-template-a-vexcode-pro-library-featuring-custom-pid-and-odometry/111793
