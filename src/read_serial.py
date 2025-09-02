import serial
import csv
import re
from datetime import datetime

# print(serial.__file__)
# print(dir(serial))


# Configurações da porta serial
PORTA = 'COM5'       # Substitua por sua porta: 'COMx' no Windows ou '/dev/ttyUSBx' no Linux
BAUDRATE = 115200
TIMEOUT = 5

# Arquivo CSV com timestamp
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
nome_arquivo = f"dados_{timestamp}.csv"

padrao = re.compile(
    r"ADC:\s*(\d+)\s*\|\s*DAC:\s*(\d+)"
)
# Abre a porta serial e o arquivo CSV
with serial.Serial(PORTA, BAUDRATE, timeout=TIMEOUT) as ser, \
    open(nome_arquivo, mode='w', newline='') as arquivo_csv:
    escritor = csv.writer(arquivo_csv)
    escritor.writerow(['ADC', 'DAC'])

    print(f"Lendo dados da porta {PORTA} e salvando em {nome_arquivo}...")

    while True:
        try:
            linha = ser.readline().decode(errors='ignore').strip()
            if not linha:
                continue

            print(linha)  # opcional: mostra a linha recebida no terminal

            # Tenta extrair os dados da linha
            match = padrao.search(linha)
            if match:
                adc = int(match.group(1))
                dac = int(match.group(2))
                escritor.writerow([adc, dac])

        except KeyboardInterrupt:
            print("\nInterrupção pelo usuário. Salvando arquivo e encerrando.")
            break
        except Exception as e:
            print(f"Erro: {e}")