#include <SpiPort.h>
SpiPort::SpiPort() {
	this->port = GPIOA;
	this->pin = 0;
}

SpiPort::SpiPort(GPIO_TypeDef *port, uint8_t pin) {
	this->port = port;
	this->pin = pin;
}

void SpiPort::setCS(GPIO_TypeDef *port, uint8_t pin) {
	this->port = port;
	this->pin = pin;
}

void SpiPort::setTx(uint8_t *dat0, uint8_t *dat1, uint8_t *dat2, uint8_t *dat3) {
	this->tx[0] = *dat0;
	this->tx[1] = *dat1;
	this->tx[2] = *dat2;
	this->tx[3] = *dat3;
	this->tx[5] = this->tx[0] + this->tx[1] + this->tx[2] + this->tx[3] + this->tx[4];
}

void SpiPort::select() {
	this->port->BRR = (1 << this->pin);
}

void SpiPort::unSelect() {
	this->port->BSRR = (1 << this->pin);
}

uint8_t SpiPort::rxSummCheck() {
	if (this->rx[0] + this->rx[1] + this->rx[2] + this->rx[3] + this->rx[4] == this->rx[5]) {
		return 1;
	}
	return 0;
}

uint8_t SpiPort::changes() {
	if (this->rxPrev[0] != this->rx[0] || this->rxPrev[1] != this->rx[1] ||
		this->rxPrev[2] != this->rx[2] || this->rxPrev[3] != this->rx[3] || this->rxPrev[4] != this->rx[4]) {

		this->rxPrev[0] = this->rx[0];
		this->rxPrev[1] = this->rx[1];
		this->rxPrev[2] = this->rx[2];
		this->rxPrev[3] = this->rx[3];
		this->rxPrev[4] = this->rx[4];
		return 1;
	}
	return 0;
}

uint8_t* SpiPort::getRx() {
	return &this->rx[0];
}
uint8_t* SpiPort::getTx() {
	return &this->tx[0];
}
