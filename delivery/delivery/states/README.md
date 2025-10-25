# Resumo dos estados

## Core.Initialize
Inicia as classes e variáveis no blackboard.
- **SUCCEED**: Conseguiu iniciar tudo.
- **ABORT**: Algo não conseguiu iniciar corretamente.

## Core.Takeoff
Realiza a decolagem e pode ou não atualizar a posição inicial (posição do RTL).
- **SUCCEED**: Conseguiu decolar e atualizar a posição inicial.
- **ABORT**: Algum elemento do blackboard não existe.

## Core.Land
Apenas pousa ou retorna para o local de início e pousa.
- **SUCCEED**: Conseguiu pousar corretamente.
- **ABORT**: Algum elemento do blackboard não existe.

## Align_pkg
Executa o yaw para alinhar com o pacote.
- **SUCCEED**: Conseguiu realizar o yaw e alinhar com o pacote.
- **FAIL**: Perdeu a detecção do pacote ou ocorreu timeout.
- **ABORT**: Algum elemento do blackboard não existe.

## CenterOnDetection
Executa centralização horizontalmente com a detecção (que pode ser o pacote ou a base).
- **SUCCEED**: Conseguiu se alinhar horizontalmente com a detecção.
- **FAIL**: Perdeu a detecção ou ocorreu timeout.
- **ABORT**: Algum elemento do blackboard não existe.

## CheckPkg
Verifica se o pacote foi coletado corretamente.
- **SUCCEED**: Não detectou mais o pacote (foi coletado).
- **FAIL**: O pacote ainda foi detectado na base.
- **CANCEL**: O pacote foi detectado fora da base.
- **ABORT**: Algum elemento do blackboard não existe.

## GoToTarget
Vai até a base ou até o pacote.
- **SUCCEED**: Conseguiu chegar ao destino.
- **ABORT**: Não conseguiu chegar ao destino ou acabaram os destinos ou algum elemento do blackboard não existe.

## GripperController
Executa a abertura ou o fechamento da garra.
- **SUCCEED**: Conseguiu controlar a garra.
- **ABORT**: Algum elemento do blackboard não existe.

## ReacquireTarget
Faz o drone subir ou descer um pouco para tentar recuperar a detecção.
- **SUCCEED**: Conseguiu atingir a altura desejada ou ocorreu timeout.
- **FAIL**: Não conseguiu atingir a altura desejada.
- **ABORT**: Algum elemento do blackboard não existe.
