/*
 * lsm9ds1_self_test.h
 *
 *  Created on: Sep 11, 2025
 *      Author: jvmen
 */

#ifndef INC_LSM9DS1_SELF_TEST_H_
#define INC_LSM9DS1_SELF_TEST_H_

#ifndef SELF_TEST_H
#define SELF_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "lsm9ds1_reg.h"

/**
 * @brief Executa o autoteste completo do LSM9DS1
 *
 * O procedimento verifica o funcionamento do magnetômetro,
 * acelerômetro e giroscópio, aplicando os testes de
 * auto-calibração definidos no datasheet.
 *
 * O resultado é transmitido pela interface serial (UART/USB)
 * conforme implementado em `tx_com`.
 *
 * @note Essa função contém laços de espera internos e só retorna
 *       ao final do teste.
 */
void lis9ds1_self_test(void);

#ifdef __cplusplus
}
#endif

#endif /* SELF_TEST_H */


#endif /* INC_LSM9DS1_SELF_TEST_H_ */
