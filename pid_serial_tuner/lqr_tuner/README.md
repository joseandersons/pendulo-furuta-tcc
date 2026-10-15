# LQR Tuner - Pêndulo de Furuta

## Instalação

```bash
pip install -r requirements.txt
```

## Execução

```bash
python main.py
```

## Comandos esperados no ESP32

- `LQR k_theta k_alpha k_theta_dot k_alpha_dot`
- `VL vel_max_rad_s accel_max_rad_s2`
- `MV omega_rad_s`
- `MSTOP`
- `RTH`
- `GET`
- `START`
- `STOP`

## Telemetria esperada

```text
T,<ms>,A,<alpha_c2>,TH,<theta_c2>,AD,<alpha_dot_c2>,TD,<theta_dot_c2>,U,<u_c2>,VEL,<vel_c2>,EN,<0/1>
```
