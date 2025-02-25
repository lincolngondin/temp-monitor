# Monitoramento da temperatura ambiente com ESP32

# Descrição
Nesse projeto foi desenvolvido um dispositivo embarcado para monitorar a temperatura ambiente e alertar caso a temperatura esteja fora do intervalo esperado.

# Funcionalidades
* Leitura da temperatura ambiente a partir do sensor DTH11.
* Exibição das informaçoes no display OLED.
* Alarme sonoro quando a temperatura ultrapassa os limites definidos.
* Botoes fisicos para interagir com o dispositivo.

# Tecnologias utilizadas
* ESP32
* FreeRTOS
* DHT11

# Montagem
Você precisará dos seguintes componentes:
* Modulo sensor de temperatura e umidade DHT11
* Plataforma embarcada ESP32-WROOM
* Display OLED I2C SSD1306 128x64
* Buzzer ativo 5v
* Três botões switch
* Protoboard para prototipação
* Três resistores de 10k

A montagem deve seguir os pinos especificados no codigo.

# Como executar
1. Instale a framework de desenvolvimento ESP-IDF seguindo o tutorial no [site](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#installation), o comando `idf.py` deve estar disponivel a partir desse passo.

2. Clone esse repositorio e mude para o diretorio criado.
```
git clone https://github.com/lincolngondin/temp-monitor.git
cd temp-monitor
```

3. Compile o projeto.
```
idf.py build
```

4. Conecte a plataforma ESP32 já montada a um computador e execute o comando para gravar o programa no ESP.
``` 
idf.py -p PORTA_AQUI flash
```

5. Caso queira monitorar os logs
```
idf.py -p PORTA_AQUI monitor
```