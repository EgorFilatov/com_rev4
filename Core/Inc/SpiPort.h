#include "main.h"

#ifndef SRC_SPIPORT_HPP_
#define SRC_SPIPORT_HPP_

class SpiPort {
private:
	uint8_t rx [6] {0};
	uint8_t tx [6] {0};
	uint8_t rxPrev [6] {0};
	uint8_t spiRxSaved [6] {0};
	uint8_t spiTxSaved [6] {0};
	GPIO_TypeDef* port;
	uint8_t pin;
	uint8_t type {0};

public:
	SpiPort();
	SpiPort(GPIO_TypeDef* port, uint8_t pin);
	void setCS(GPIO_TypeDef *port, uint8_t pin);
	void setTx(uint8_t *dat0, uint8_t *dat1, uint8_t *dat2, uint8_t *dat3);
	void select();
	void unSelect();
	uint8_t rxSummCheck();
	uint8_t changes();
	uint8_t* getRx();
	uint8_t* getTx();
};

#endif
