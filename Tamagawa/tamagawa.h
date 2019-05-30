#ifndef TAMAGAWA_H
#define TAMAGAWA_H

#include "stm32f4xx.h"

/**
 *    @brief    Data ID codes
 */
enum DATA_ID
{
    DATA_ID_0,  /**< Data readout data in one revolution */
    DATA_ID_1,  /**< Data readout multi-turn data */
    DATA_ID_2,  /**< Data readout encoder ID */
    DATA_ID_3,  /**< Data readout data in one revolution, encoder ID, multi-turn, encoder error */
    DATA_ID_6,  /**< EEPROM write */
    DATA_ID_7,  /**< Reset */
    DATA_ID_8,  /**< Reset */
    DATA_ID_C,  /**< Reset */
    DATA_ID_D,  /**< EEPROM read */
    DATA_ID_NUM /**< Number of Data ID codes */
};

/**
 * @brief Tamagawa Interface Transmit data
 */
struct tx
{
    uint8_t  adf;   /**< EEPROM address */
    uint8_t  edf;   /**< EEPROM data */
};

/**
 * @brief Tamagawa Interface Received data
 */
struct rx
{
    uint32_t abs;   /**< Data in one revolution */
    short abm;   /**< Multi-turn Data */
    uint8_t  cf;    /**< Control Frame */
    uint8_t  sf;    /**< Status Frame */
    uint8_t  enid;  /**< Encoder ID */
    uint8_t  almc;  /**< Encoder error */
    uint8_t  adf;   /**< EEPROM address */
    uint8_t  edf;   /**< EEPROM data */
    uint8_t  crc;   /**< CRC */
};

/**
 * @brief Tamagawa Interface
 */
struct TamagawaInterface
{
    uint32_t version;  /**< Firmware version */
    uint8_t  config;   /**< Configuration */
    uint8_t  data_id;  /**< Data ID code */
    struct tx tx;      /**< Transmit data */
    struct rx rx;      /**< Received data */
};

extern uint8_t usart_data;
extern struct TamagawaInterface ti;

void tamagawa_read(uint8_t data_id);

#endif
