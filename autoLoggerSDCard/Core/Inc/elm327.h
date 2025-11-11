/*
 * elm327.h
 *
 *  Created on: Nov 9, 2025
 *      Author: jvmen
 */

#ifndef INC_ELM327_H_
#define INC_ELM327_H_

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* INCLUDES ----------------------------------------------------------------- */
#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* -------------------------------------------------------------------------- */
/* DEFINES ------------------------------------------------------------------ */

/**
 * @brief  Tamanho máximo do buffer de recepção UART (para dados do ELM327)
 */
#define UART_RX_BUFFER_SIZE 1024

/* -------------------------------------------------------------------------- */
/* VARIÁVEIS GLOBAIS -------------------------------------------------------- */

/* Buffers e flags compartilhados entre o módulo e o código principal */
extern uint8_t  uart_rx_byte;                               /**< Último byte recebido via UART */
extern uint8_t  uart_rx_buffer[UART_RX_BUFFER_SIZE];         /**< Buffer de armazenamento de dados */
extern volatile uint16_t uart_rx_index;                      /**< Posição atual de escrita no buffer */
extern volatile uint8_t  uart_data_ready;                    /**< Flag: indica que há dados completos disponíveis */
extern volatile uint8_t  uart_response_count;                /**< Contador de respostas completas */
extern volatile uint8_t  uart_obd_byte_count;                /**< Contador de bytes da resposta atual */

/* -------------------------------------------------------------------------- */
/* PROTÓTIPOS DE FUNÇÕES ---------------------------------------------------- */

/**
 * @brief  Inicializa o módulo ELM327 e envia comandos básicos de configuração.
 * @param  huart: ponteiro para a UART conectada ao ELM327 (geralmente &huart1)
 * @retval Nenhum
 */
void ELM327InitDebug(UART_HandleTypeDef *huart);

/**
 * @brief  Converte uma string hexadecimal (ex: "1A") em byte.
 * @param  hex: ponteiro para os dois caracteres hexadecimais.
 * @retval Valor convertido em byte.
 */
uint8_t hexToByte(const char *hex);

/**
 * @brief  Envia um comando OBD-II (PID) via UART1 para o ELM327.
 * @param  pid: string do comando, sem o '\r' (ex: "010C").
 * @retval Nenhum
 */
void ELM327SendCommand(const char *pid);

/**
 * @brief  Função a ser chamada dentro do callback UART (HAL_UART_RxCpltCallback).
 *         Ela processa cada byte recebido e organiza as respostas completas.
 * @retval Nenhum
 */
void ELM327UARTCallback(void);

/**
 * @brief  Função placeholder para leitura e tratamento dos dados OBD.
 *         Pode ser expandida conforme a necessidade de análise.
 * @retval Nenhum
 */
void OBDReadData(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ELM327_H_ */
