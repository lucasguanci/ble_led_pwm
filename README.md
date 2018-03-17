# BLE led stripe
Bluetooth Low Energy app based on Apache Mynewt OS to drive an RGB led stripe using hardware PWM on a nRF52 Development Kit.

The app is based on the [BLE peripheral project](https://mynewt.apache.org/latest/os/tutorials/bleprph/bleprph-intro/) and uses PWM to control the color of an RGB led stripe.
The app uses mynewt master branch.
The target is set as follows:
```
targets/ble_led_pwm_tgt
    app=apps/ble_led_pwm
    bsp=@apache-mynewt-core/hw/bsp/nrf52dk
    build_profile=debug
    syscfg=PWM_0=1
```
