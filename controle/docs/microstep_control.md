# Controle de microsteps

A lógica de troca de microsteps do LQR foi movida para `include/microstep_control.h` e `src/microstep_control.cpp`.

Comandos seriais adicionados:

- `MSAUTO 1`: liga controle automático de microsteps.
- `MSAUTO 0`: desliga controle automático e passa a usar o microstep manual.
- `MSMAN <valor>`: define microstep manual. Valores aceitos: `8`, `16`, `32`, `64`, `128`, `256`.
- `MS?`: mostra status atual.

Quando o automático está ligado, o comportamento antigo é mantido:

- perto do equilíbrio: microstep fino `256` e ganhos LQR da interface;
- distúrbio: microstep `16` e perfil de ganhos de distúrbio.

Quando o automático está desligado, o firmware aplica o microstep manual escolhido e usa os ganhos LQR normais da interface.
