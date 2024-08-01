import matplotlib.pyplot as plt

# Função para ler valores de um arquivo txt
def ler_valores(arquivo):
    with open(arquivo, 'r') as f:
        valores = f.readlines()
    # Converter valores para float e remover a vírgula no final
    valores = [float(valor.strip().rstrip(',')) for valor in valores]
    return valores

# Caminhos dos arquivos de entrada
arquivo_x = 'deslocamento_x.txt'
arquivo_y = 'deslocamento_y.txt'

# Ler valores dos arquivos
valores_x = ler_valores(arquivo_x)
valores_y = ler_valores(arquivo_y)

# Verificar se os arquivos têm o mesmo número de valores
if len(valores_x) != len(valores_y):
    raise ValueError("Os arquivos de deslocamento X e Y devem ter o mesmo número de valores.")

# Plotar o gráfico de deslocamento
plt.figure(figsize=(10, 6))
plt.plot(valores_x, valores_y, marker='o')
plt.title('Gráfico de Deslocamento nos Eixos X e Y')
plt.xlabel('Deslocamento em X')
plt.ylabel('Deslocamento em Y')
plt.grid(True)
plt.show()
