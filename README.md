# DWA_RealSense
Código de desvio de obstáculo desenvolvido utilizando o método de janela dinâmica (DWA)

Neste repositório, será publicado um pacote ROS, junto à um código de desvio de obstáculo utilizando o método de Janela Dinâmica (Dynamic Windown Approach), em uma simulação num ambiente gerado pelo Gazebo, utilizando do sistema de robótica operacional (ROS) para melhor eficiência na simulação.

Para utilizar o pacote, é necessário, primeiramente fazer a instalação. Para isso, digite no seu terminal, mais especificamente na sua pásta de trabalho catkin, o comando abaixo:

Feito isso, digite os comandos de "catkin_make" e "devel/setup.bash" para atualizar sua pasta de trabalho. 

Por fim, digite o comando para lançar o mundo e o robô de simulação Nexus, o qual foi utilizado para este projeto.

Feito isso, será necessário a instalação do Python3 na sua máquina. A instalação pode ser feita neste link a seguir: https://www.python.org/downloads/
Após a instalação do Python3, com o terminal, abra a pasta "src", aonde se encontra o código de DWA. Para utilizá-lo, digite o comando "python3 dwa.py". Com isso, aparecerá no terminal a opção para determinar as coordenadas x e y de ponto final do robô. Escolhendo as coordenadas, o robô vai se direcionar até o ponto desejado, e esquivar dos obstáculos que estiverem no caimnho.
