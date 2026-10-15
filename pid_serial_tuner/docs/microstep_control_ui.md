# Interface de microsteps

A tela `lqr_tuner/main.py` ganhou a seção `Microsteps do LQR`.

Ela permite:

- ligar/desligar o controle automático de microsteps;
- selecionar o microstep manual entre `8`, `16`, `32`, `64`, `128` e `256`;
- aplicar a configuração via serial com os comandos `MSMAN`, `MSAUTO` e `MS?`.

Ao clicar em `INICIAR`, a configuração atual de microsteps é enviada antes do comando `START`.
