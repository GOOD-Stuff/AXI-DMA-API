#include "axidma.h"


inline int	axidma_soft_set_tx_threshold(axidma_t *dmainstptr, int thres) {
	return dmainstptr->status_tx |= (thres << THRESHOLD_SHIFT) & DMACR_THRESHOLD_MASK;
}


inline int	axidma_soft_set_rx_threshold(axidma_t *dmainstptr, int thres) {
	return dmainstptr->status_rx |= (thres << THRESHOLD_SHIFT) & DMACR_THRESHOLD_MASK;
}


inline int axidma_soft_set_tx_delay(axidma_t *dmainstptr, int delay) {
	return dmainstptr->status_tx |= (delay << DELAY_SHIFT) & DMACR_DELAY_MASK;
}


inline int axidma_soft_set_rx_delay(axidma_t *dmainstptr, int delay) {
	return dmainstptr->status_rx |= (delay << DELAY_SHIFT) & DMACR_DELAY_MASK;
}


inline int axidma_soft_set_tx_irq(axidma_t *dmainstptr, int irq_vector) {
	return dmainstptr->status_tx |= (irq_vector >> IRQ_SOFT_OFFSET);
}


inline int	axidma_soft_set_rx_irq(axidma_t *dmainstptr, int irq_vector) {
	return dmainstptr->status_rx |= (irq_vector >> IRQ_SOFT_OFFSET);
}


inline int axidma_soft_get_tx_threshold(axidma_t *dmainstptr) {
	return ((dmainstptr->status_tx & DMACR_THRESHOLD_MASK) >> THRESHOLD_SHIFT);
}


inline int	axidma_soft_get_rx_threshold(axidma_t *dmainstptr) {
	return ((dmainstptr->status_rx & DMACR_THRESHOLD_MASK) >> THRESHOLD_SHIFT);
}


inline int axidma_soft_get_tx_delay(axidma_t *dmainstptr) {
	return ((dmainstptr->status_tx & DMACR_DELAY_MASK) >> DELAY_SHIFT);
}


inline int axidma_soft_get_rx_delay(axidma_t *dmainstptr) {
	return ((dmainstptr->status_rx & DMACR_DELAY_MASK) >> DELAY_SHIFT);
}


inline int axidma_soft_get_tx_irq(axidma_t *dmainstptr) {
	return((dmainstptr->status_tx & IRQ_SOFT_MASK) << IRQ_SOFT_OFFSET);
}


inline int axidma_soft_get_rx_irq(axidma_t *dmainstptr) {
	return ((dmainstptr->status_rx & IRQ_SOFT_MASK) << IRQ_SOFT_OFFSET);
}


/**
 * @brief Инициализация устройства AXI DMA.
 * Выполняется открытие дескриптора у управляющей структуры AXI DMA
 * а также выделение памяти под аппаратный AXI DMA в виде отображения на память
 * @param[out] dmainstptr     - handle of axidma
 * @param[in]  axidma_address - address of physical AXI DMA
 *
 * @return
 * 		- ERR_DMA_DEV_BAD_ADDRESS, если адрес AXI DMA не соответствует диапазону AXI
 * 		- ERR_DMA_OPEN_FD, если произошла ошибка открытия дескриптора файла
 * 		- ERR_DMA_DEV_MAP, если произошла ошибка отображения устройства на память
 * 		- DMA_OK, если все прошло нормально
 * @note	также инициализирует базовыми значениями структуру bdring
 *
 * @date 05.04.2017
 * @author Chemodanov Maxim
 */
int axidma_initialization(axidma_t *dmainstptr, uint32_t axidma_address) {
	dmainstptr->Initialized = 0;

	if ((axidma_address < 0x40000000) || (axidma_address > 0xC0000000))
		return ERR_DMA_DEV_BAD_ADDRESS;

	dmainstptr->baseaddr_hw         = axidma_address;
	dmainstptr->dma_file_descriptor = open("/dev/mem", O_RDWR | O_SYNC);
	if (dmainstptr->dma_file_descriptor < 0)
		return ERR_DMA_OPEN_FD;

	dmainstptr->dma_hw = (dma_device_t*)mmap(NULL, 0xFFFF, PROT_READ | PROT_WRITE,
			                                 MAP_SHARED,
										     dmainstptr->dma_file_descriptor,
										     dmainstptr->baseaddr_hw);
	if (dmainstptr->dma_hw == MAP_FAILED)
		return ERR_DMA_DEV_MAP;

	dmainstptr->isRx 	  = 0;
	dmainstptr->isTx 	  = 0;
	dmainstptr->status_rx = 0;
	dmainstptr->status_tx = 0;

	if (axidma_hasrx(dmainstptr)) dmainstptr->isRx = 1;
	if (axidma_hastx(dmainstptr)) dmainstptr->isTx = 1;

	dmainstptr->Initialized = 1;

	return DMA_OK;
}


/**
 * Возвращает состояние инициализации программного AXI DMA
 * @param 	- указатель
 * @return
 * 		- 1, если инициализирован
 * 		- 0, если axidma_t не инициализирован
 * @note	- NONE
 * @date	- 06.04.2017
 * @author	-
 */
inline int axidma_initialized(axidma_t *dmainstptr) {
	return (dmainstptr->Initialized) ? 1 : 0;
}


/*
 * @brief Узнать, является ли AXI DMA сконфигурированным в SG MODE
 * @param 	- указатель на axidma
 * @return
 * 		- 0, если AXI DMA не в SG режиме
 * 		- other, если AXI DMA в SG режиме
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
inline int	axidma_is_sg(axidma_t *dmainstptr) {
	uint32_t rx_status_reg = 0;
	uint32_t tx_status_reg = 0;

	if (axidma_hasrx(dmainstptr)){
		rx_status_reg = axidma_get_rx_status(dmainstptr) & DMASR_ISSG_MASK;
	}

	if (axidma_hastx(dmainstptr)){
		tx_status_reg = axidma_get_tx_status(dmainstptr) & DMASR_ISSG_MASK;
	}

	return rx_status_reg + tx_status_reg;
}


/*
 * Получение статусного регистра с аппаратного AXI DMA
 * @param - указатель на axidma_t структуру
 * @return - статусный регистр аппаратного AXI DMA
 * @note - none
 * */
inline uint32_t axidma_get_tx_status(axidma_t *dmainstptr) {
	return dmainstptr->dma_hw->mm2s_dmasr;
}


/*
 * @brief Читает статусный регистр с аппаратного AXI DMA на канале приема
 * @param 	- указатель на axidma
 * @return
 * 		- статусный регистр канала приема данных аппаратного axi dma
 * @note	- NONE
 * @date	- 06.04.2017
 * @author	-
 * */
inline uint32_t axidma_get_rx_status(axidma_t *dmainstptr) {
	return dmainstptr->dma_hw->s2mm_dmasr;
}


/*
 * @brief возвращает контрольный регистр axi dma канала передачи
 * @param 		указатель на axidma
 * @return
 * 		- контрольный регистр аппаратного axi dma канала передачи
 * @note		NONE
 * @date		06.04.2017
 * @author	-
 * */
inline uint32_t axidma_get_tx_control(axidma_t *dmainstptr) {
	return dmainstptr->dma_hw->mm2s_dmacr;
}


/*
 * возвращает контрольный регистр axidma_t канала приема
 * @param 	- указатель на axidma
 * @return
 * 		- контрольный регистр аппаратного axi dma канала приема
 * @note		NONE
 * @date	-	06.04.2017
 * @author	-
 * */
inline uint32_t axidma_get_rx_control(axidma_t *dmainstptr) {
	return dmainstptr->dma_hw->s2mm_dmacr;
}


/*
 * @brief Возвращает регистр адреса текущего дескриптора axidma_t канала передачи данных
 * @param 	- указатель на axidma
 * @return
 * 		- регистр адреса текущего дескриптора axidma_t канала передачи
 * @note	NONE
 * @date	06.04.2017
 * @author Chemodanov Maxim
 * */
inline uint32_t axidma_get_tx_curdesc(axidma_t *dmainstptr) {
	return dmainstptr->dma_hw->mm2s_curdesc;
}


/*
 * @brief Возвращает регистр адреса текущего дескриптора axidma_t канала приема данных
 * @param 	- указатель на axidma
 * @return
 * 		- регистр адреса текущего дескриптора axidma_t канала приема
 * @note	NONE
 * @date	06.04.2017
 * @author Chemodanov Maxim
 * */
inline uint32_t axidma_get_rx_curdesc(axidma_t *dmainstptr) {
	return dmainstptr->dma_hw->s2mm_curdesc;
}


/*
 * @brief Возвращает регистр адреса последнего дескриптора axidma_t канала передачи данных
 * @param 	- указатель на axidma
 * @return
 * 		- регистр адреса последнего дескриптора axidma_t канала передачи
 * @note	NONE
 * @date	06.04.2017
 * @author Chemodanov maxim
 * */
inline uint32_t axidma_get_tx_taildesc(axidma_t *dmainstptr) {
	return dmainstptr->dma_hw->mm2s_taildesc;
}


/*
 * @brief Возвращает регистр адреса последнего дескриптора axidma_t канала приема данных
 * @param 	- указатель на axidma
 * @return
 * 		- регистр адреса последнего дескриптора axidma_t канала приема
 * @note	NONE
 * @date	06.04.2017
 * @author	-
 * */
inline uint32_t axidma_get_rx_taildesc(axidma_t *dmainstptr) {
	return dmainstptr->dma_hw->s2mm_taildesc;
}


/*
 * @brief Установка статусного регистра канала передачи axidma
 * @param 	- указатель на axidma-структуру
 * @param 	- значение статусного регистра
 * @return
 * 		- DMA_OK
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_set_tx_status(axidma_t *dmainstptr, uint32_t status_reg) {
	dmainstptr->dma_hw->mm2s_dmasr = status_reg;
	return DMA_OK;
}


/*
 * Установка статусного регистра канала поиема у axidma
 * @param 	- указатель на axidma-структуру
 * @param 	- значение статусного регистра
 * @return
 * 		- DMA_OK
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_set_rx_status(axidma_t *dmainstptr, uint32_t status_reg) {
	dmainstptr->dma_hw->s2mm_dmasr = status_reg;
	return DMA_OK;
}


/*
 * Установка контрольного регистра канала передачи у axidma
 * @param 	- указатель на axidma-структуру
 * @param 	- значение контрольного регистра канала передачи axidma
 * @return
 * 		- DMA_OK
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_set_tx_control(axidma_t *dmainstptr, uint32_t control_reg) {
	dmainstptr->dma_hw->mm2s_dmacr = control_reg;
	return DMA_OK;
}


/*
 * Установка контрольного регистра канала приема у axidma
 * @param 	- указатель на axidma-структуру
 * @param 	- значение контрольного регистра канала приема axidma
 * @return
 * 		- DMA_OK
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_set_rx_control(axidma_t *dmainstptr, uint32_t control_reg) {
	dmainstptr->dma_hw->s2mm_dmacr = control_reg;
	return DMA_OK;
}



/*
 * Установка значения адреса текущего дескриптора в регистр канала передачи у axidma
 * @param 	- указатель на axidma-структуру
 * @param 	- значение адреса текущего дескриптора
 * @return
 * 		- DMA_OK
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
void axidma_set_tx_curdesc(axidma_t *dmainstptr, uint32_t curdesc_reg) {
	dmainstptr->dma_hw->mm2s_curdesc = curdesc_reg;
}


/*
 * Установка значения адреса текущего дескриптора в регистр канала приема у axidma
 * @param 	- указатель на axidma-структуру
 * @param 	- значение адреса текущего дескриптора
 * @return
 * 		- DMA_OK
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
void axidma_set_rx_curdesc(axidma_t *dmainstptr, uint32_t curdesc_reg) {
	dmainstptr->dma_hw->s2mm_curdesc = curdesc_reg;
}


/*
 * Установка значения адреса последнего дескриптора в регистр канала передачи у axidma
 * @param 	- указатель на axidma-структуру
 * @param 	- значение адреса последнего дескриптора
 * @return
 * 		- DMA_OK
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
void axidma_set_tx_taildesc(axidma_t *dmainstptr, uint32_t taildesc_reg) {
	dmainstptr->dma_hw->mm2s_taildesc = taildesc_reg;
}


/*
 * Установка значения адреса последнего дескриптора в регистр канала приема у axidma
 * @param 	- указатель на axidma-структуру
 * @param 	- значение адреса последнего дескриптора
 * @return
 * 		- DMA_OK
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
void axidma_set_rx_taildesc(axidma_t *dmainstptr, uint32_t taildesc_reg) {
	dmainstptr->dma_hw->s2mm_taildesc = taildesc_reg;
}


/*
 * @brief Проверка структуры на инициализацию и запуск канала передачи axidma
 *
 * @param 	- указатель на axidma
 *
 * @return
 * 		- ERR_DMA_UNINIT, если программный DMA не инициализирован
 * 		- DMA_OK, если проинициализировано
 *
 * @note		Перед выполнением данной функции убедитесь в том, что в регистр
 * аппаратного axi dma было записано значение адреса текущего дескриптора.
 * После вызова данной функции рекомендуется записать taildesc - значение адреса
 * последнего дескриптора.
 * @date	- 	06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_start(axidma_t *dmainstptr) {
	if (!axidma_initialized(dmainstptr)) {
		return ERR_DMA_UNINIT;
	}

	if (!axidma_tx_iswork(dmainstptr)) {
		uint32_t control_reg = axidma_get_tx_control(dmainstptr) | DMACR_RS_MASK;
		axidma_set_tx_control(dmainstptr, control_reg);
	}

	return DMA_OK;
}

/*
 * Проверка структуры на инициализацию и запуск канала приема axidma
 *
 * @param 	- указатель на axidma
 *
 * @return
 * 		- ERR_DMA_UNINIT, если программный DMA не инициализирован
 * 		- DMA_OK, если проинициализировано
 *
 * @note		Перед выполнением данной функции убедитесь в том, что в регистр
 * аппаратного axi dma было записано значение адреса текущего дескриптора.
 * После вызова данной функции рекомендуется записать taildesc значение адреса
 * последнего дескриптора.
 * @date	- 	06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_start (axidma_t *dmainstptr) {
	if (!axidma_initialized(dmainstptr)){
		return ERR_DMA_UNINIT;
	}

	if (!axidma_rx_iswork(dmainstptr)){
		uint32_t control_reg = axidma_get_rx_control(dmainstptr) | DMACR_RS_MASK;
		axidma_set_rx_control(dmainstptr, control_reg);
	}

	return DMA_OK;
}


uint32_t axidma_api_start (axidma_t *dmainstptr) {
	if (axidma_api_hasrx(dmainstptr)) {
		bdring_t *rxring = axidma_get_rxring(dmainstptr);
		if (bdring_get_state(rxring) == BDRING_UNINIT_STATE)
			return ERR_RING_UNINIT;

		axidma_set_rx_curdesc(dmainstptr, rxring->bd_headaddr);
		int status = axidma_rx_start(dmainstptr);
		if (status != DMA_OK)
			return status;

		int delay_timer = RESET_TIMEOUT;
		while (!axidma_rx_iswork(dmainstptr)){
			delay_timer--;
			if (delay_timer == 0)
				return ERR_DMA_HAD_WORK;
		}

		axidma_set_rx_taildesc(dmainstptr, rxring->bd_lastaddr);
	}

	return DMA_OK;
}


/*
 * @brief Приостанавливает поток axidma_t на канале передачи данных
 *
 * @param 	- указатель на axidma
 *
 * @return
 * 			- 	ERR_DMA_UNINIT, если дма не был проинициализирован
 * 			- 	DMA_OK, если запрос на остановку был отправлен
 *
 * @note	- 	стоит помнить, что после выполнения данной функции и
 * 				реальной остановкой axi dma существует небольшая пауза.
 * 				за эту паузу процессор успевает провести 70 операций сложения.
 * 				если момент остановки является крайне важным,
 * 				необходимо перейти в режим ожидания до
 * 				тех пор, пока дма полностью не остановится.
 * 				В противном случае при анализе дескрипторов
 * 				можно получить неверные данные о текущем состоянии дескрипторов.
 * 				Это справедливо для rx канала. Для tx это не проверялось
 *
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_pause(axidma_t *dmainstptr) {
	if (!axidma_initialized(dmainstptr)){
		return ERR_DMA_UNINIT;
	}

	if (axidma_tx_iswork(dmainstptr)){
		uint32_t control_reg = axidma_get_tx_control(dmainstptr) & ~DMACR_RS_MASK;
		axidma_set_tx_control(dmainstptr, control_reg);
	}

	return DMA_OK;
}


/*
 * Приостанавливает поток axidma_t на канале приема данных
 *
 * @param 	- указатель на axidma
 *
 * @return
 * 		- ERR_DMA_UNINIT, если дма не был проинициализирован
 * 		- DMA_OK, если запрос на остановку был отправлен
 *
 * @note	- стоит помнить, что после выполнения данной функции и
 * реальной остановкой axi dma существует небольшая пауза.
 * за эту паузу процессор успевает провести 70 операций сложения
 * (Fclk = 125 MHz, DataWidth = 32 bit, speed - 420 MByte).
 * если момент остановки является крайне важным, необходимо перейти в режим ожидания до
 * тех пор, пока дма полностью не остановится.
 * (для этого есть функция axidma_pauseisdone())
 * В противном случае при анализе дескрипторов
 * можно получить неверные данные о текущем состоянии дескрипторов.
 * Это справедливо для rx канала. Для tx это не проверялось.
 * Момент не указанный в документации - при принудительной остановке
 * если источник данных способен писать данные в память через ДМА,
 * а дма содержит в своей структуре свободные дескрипторы, дозаписывается два дескриптора,
 * после этого ДМА переходит в состояние ожидания.
 *
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_pause(axidma_t *dmainstptr) {
	uint32_t control_reg = 0;

	if (!axidma_initialized(dmainstptr)) {
		return ERR_DMA_UNINIT;
	}

	if (axidma_rx_iswork(dmainstptr)) {
		control_reg = axidma_get_rx_control(dmainstptr) & ~DMACR_RS_MASK;
		axidma_set_rx_control(dmainstptr, control_reg);
	}

	return DMA_OK;
}


/*
 * @brief Возвращает состояние канала передачи axidma
 * @param 	- указатель на AXI DMA
 * @return
 * 		- 0, если канал остановлен(halted)
 * 		- 1, если канал запущен(run)
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_iswork(axidma_t *dmainstptr) {
	return (dmainstptr->dma_hw->mm2s_dmasr & DMASR_HALT_MASK) ? 0 : 1;
}


/*
 * Возвращает состояние канала приема axidma
 * @param 	- указатель на AXI DMA
 * @return
 * 		- 0, если канал остановлен(halted)
 * 		- 1, если канал запущен(run)
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_iswork(axidma_t *dmainstptr) {
	return (dmainstptr->dma_hw->s2mm_dmasr & DMASR_HALT_MASK) ? 0 : 1;
}


/*
 * @brief Возвращает состояние окончания выполнения паузы на канале
 * передачи данных axi dma
 * @param 	- указатель на axi dma
 * @return
 * 		- 1, если axi dma перешел в состояние halted
 * 		- 0, если axi dma находится в рабочем состоянии run
 * @note		NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_pauseisdone(axidma_t *dmainstptr) {
	return (dmainstptr->dma_hw->mm2s_dmasr & DMASR_HALT_MASK) ? 1 : 0;
}


/*
 * Возвращает состояние окончания выполнения паузы на канале
 * приема данных axi dma
 * @param 	- указатель на axi dma
 * @return
 * 		- 1, если axi dma перешел в состояние halted
 * 		- 0, если axi dma находится в рабочем состоянии run
 * @note		NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_pauseisdone(axidma_t *dmainstptr) {
	return (dmainstptr->dma_hw->s2mm_dmasr & DMASR_HALT_MASK) ? 1 : 0;
}


/*
 * @brief Возобновляет работу канала передачи данных на аппаратном AXI DMA
 * @param 	- указатель на AXI DMA
 * @return
 * 		- ERR_DMA_UNINIT, если AXI DMA не проинициализирован
 * 		- DMA_OK, если на AXI DMA отправлен запрос на возобновление
 * 		работы
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_resume(axidma_t *dmainstptr) {
	if (!axidma_initialized(dmainstptr)){
		return ERR_DMA_UNINIT;
	}

	if (!axidma_tx_iswork(dmainstptr)){
		uint32_t control_reg = axidma_get_tx_control(dmainstptr) | DMACR_RS_MASK;
		axidma_set_tx_control(dmainstptr, control_reg);
	}

	return DMA_OK;
}


/*
 * @brief Возобновляет работу канала приема данных на аппаратном AXI DMA
 * @param 	- указатель на AXI DMA
 * @return
 * 		- ERR_DMA_UNINIT, если AXI DMA не проинициализирован
 * 		- DMA_OK, если на AXI DMA отправлен запрос на возобновление
 * 		работы
 * @note	- 	перед возобновлением работы убедитесь, что
 * 				все операции с curdesc/taildesc проделаны
 * 				корректно как на аппаратном, так и на программном уровне
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_resume(axidma_t *dmainstptr) {
	if (!axidma_initialized(dmainstptr)){
		return ERR_DMA_UNINIT;
	}

	if (!axidma_tx_iswork(dmainstptr)){
		uint32_t control_reg = axidma_get_tx_control(dmainstptr) | DMACR_RS_MASK;
		axidma_set_tx_control(dmainstptr, control_reg);
	}

	return DMA_OK;
}


/*
 * @brief Выполняет сброс обоих каналов на аппаратном AXI DMA.
 * Рекомендуется запускать при каждом старте программы
 * так как AXI DMA  своих регистрах может содержать некорректные данные.
 * Сброс решает эту проблему, так как переводит регистры AXI DMA в исходное состояние.
 * @param 	-	указатель на axi dma
 * @return
 * 		- ERR_DMA_RESET_RX, если возникла проблема на сбросе канала RX
 * 		- ERR_DMA_RESET_TX, если возникла проблема на сбросе канала TX
 * 		- DMA_OK, если сброс выполнен
 * @note	В функции фигурирует таймер сброса. Если в течение этого времени сброс не будет
 * произведен, выполнение функции закончится с ошибкой
 * @date	-	15.04.2017
 * @author	-
 * */
uint32_t axidma_api_reset(axidma_t *dmainstptr) {
	int status = 0;
	int delay_counter = RESET_TIMEOUT;

	/*rx channel reset*/
	if (axidma_api_hasrx(dmainstptr)) {
		status = axidma_rx_reset(dmainstptr);
		if (status != DMA_OK)
			return ERR_DMA_RESET_RX;

		while (!axidma_rx_resetisdone(dmainstptr)) {
			delay_counter--;
			if (delay_counter == 0)
				return ERR_DMA_RESET_RX;
		}
	}

	/*tx channel reset*/
	if (axidma_api_hastx(dmainstptr)) {
		delay_counter = RESET_TIMEOUT;
		status        = axidma_tx_reset(dmainstptr);
		if (status != DMA_OK) {
			return ERR_DMA_RESET_TX;
		}
		while (!axidma_tx_resetisdone(dmainstptr)) {
			delay_counter--;
			if (delay_counter == 0)
				return ERR_DMA_RESET_TX;
		}
	}
	return DMA_OK;
}
////////////////////////////////////////////////////////////////////////////////


/*
 * @brief Сброс канала передачи у AXI DMA
 * @param 	- указатель на AXI DMA
 * @return
 * 		- ERR_DMA_UNINIT, если AXI DMA не был проинициализирован
 * @note 	- после сброса все регистры аппаратного AXI DMA переходят в
 * 			базовое состояние на канале передачи
 * 			Для того чтобы проверить завершение сброса необходимо вызвать функцию
 * 			dma_tx_resetisdone()
 * @date	-	06.04.2017
 * @author Chemodanov Maxim
 * */
uint32_t axidma_tx_reset(axidma_t *dmainstptr) {
	uint32_t control_reg = 0;

	if (!axidma_initialized(dmainstptr))
		return ERR_DMA_UNINIT;

	control_reg = axidma_get_tx_control(dmainstptr) & DMACR_RESET_MASK;
	axidma_set_tx_control(dmainstptr, control_reg);

	return DMA_OK;
}


/*
 * @brief Сброс канала приема у AXI DMA
 * @param 	- указатель на AXI DMA
 * @return
 * 		- ERR_DMA_UNINIT, если AXI DMA не был проинициализирован
 * 		- DMA_OK, если был дан сигнал сброса
 * @note 	- после сброса все регистры аппаратного AXI DMA переходят в
 * 			базовое состояние на канале приема данных
 * 			Для того чтобы проверить завершение сброса необходимо вызвать функцию
 * 			dma_rx_resetisdone()
 * @date	-	06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_reset(axidma_t *dmainstptr) {
	uint32_t control_reg = 0;
	int status = 0;
	if (!axidma_initialized(dmainstptr)){
		return ERR_DMA_UNINIT;
	}

	control_reg = axidma_get_rx_control(dmainstptr) | DMACR_RESET_MASK;

	status = axidma_set_rx_control(dmainstptr, control_reg);
	if (status != DMA_OK){
		return -1;
	}
	return DMA_OK;

}

/*
 * @brief Функция возвращает состояние операции сброса для канала tx
 * @param 	- указатель на AXI DMA
 * @return
 * 		- 1, если сброс окончен
 * 		- 0, если AXI DMA находится в процессе сброса
 * @note	Программная структура AXI DMA при этом остается при своих параметрах
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_resetisdone(axidma_t *dmainstptr) {
	return (axidma_get_tx_control(dmainstptr) & DMACR_RESET_MASK) ? 0 : 1;
}


/*
 * @brief Функция возвращает состояние операции сброса для канала rx
 * @param 	- указатель на AXI DMA
 * @return
 * 		- 1, если сброс окончен
 * 		- 0, если AXI DMA находится в процессе сброса
 * @note	Программная структура AXI DMA при этом остается при своих параметрах
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_resetisdone(axidma_t *dmainstptr) {
	return (axidma_get_rx_control(dmainstptr) & DMACR_RESET_MASK) ? 0 : 1;
}


/*
 * Возвращает текущий статус канала передачи
 * @param 	- указатель на AXI DMA
 * @return
 * 		- 0, если канал находится в состоянии IDLE
 * 		- 1, если канал работает в режиме приема данных
 * @note	- IDLE срабатывает при достижении currentdesc = taildesc
 * @date	- 06.07.2017
 * @author	-
 * */
uint32_t axidma_tx_busy(axidma_t *dmainstptr) {
	return (axidma_get_tx_status(dmainstptr) & DMASR_IDLE_MASK) ? 0 : 1;
}


/*
 * Возвращает текущий статус канала приема
 * @param 	- указатель на AXI DMA
 * @return
 * 		- 0, если канал находится в состоянии IDLE
 * 		- 1, если канал работает в режиме приема данных
 * @note	- 	IDLE срабатывает при достижении currentdesc = taildesc
 * 				ограничение на Burst AXI: не более 16
 * 				busy = axi dma выполняет какую-либо работу
 * 				!busy = axidma_t ничего не делает и ожидает данных с канала приема.
 * @date	- 06.07.2017
 * @author	-
 * */
uint32_t axidma_rx_busy(axidma_t *dmainstptr) {
	return (axidma_get_rx_status(dmainstptr) & DMASR_IDLE_MASK) ? 1 : 0;
}


/*
 * Установить режим KeyHole на канал передачи
 * @param - указатель на AXI DMA;
 * @return
 * 		- ERR_DMA_UNINIT,	если AXI DMA е инициализирован
 * 		- DMA_OK,
 * @note	burst должен быть < 16, AXI DMA должен находиться в состоянии IDLE
 * @date	06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_setKeyholeOn	(axidma_t *dmainstptr) {
	if (!axidma_initialized(dmainstptr)){
		return ERR_DMA_UNINIT;
	}

	if (!axidma_tx_busy(dmainstptr)){
		uint32_t control_reg = axidma_get_tx_control(dmainstptr) | DMACR_KEYHOLE_MASK;
		axidma_set_tx_control(dmainstptr, control_reg);
	}

	return DMA_OK;
}


/*
 * Установить режим KeyHole на канал приема
 * @param - указатель на AXI DMA;
 * @return
 * 		- ERR_DMA_UNINIT,	если AXI DMA е инициализирован
 * 		- DMA_OK, если keyhole установлен
 * @note	burst должен быть < 16, AXI DMA должен находиться в состоянии IDLE
 * @date	06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_setKeyholeOn	(axidma_t *dmainstptr) {
	if (!axidma_initialized(dmainstptr)){
		return ERR_DMA_UNINIT;
	}

	if (!axidma_rx_busy(dmainstptr)){
		uint32_t control_reg = axidma_get_rx_control(dmainstptr) | DMACR_KEYHOLE_MASK;
		axidma_set_rx_control(dmainstptr, control_reg);
	}

	return DMA_OK;
}


/*
 * Выключает режим KeyHole на канал передачи
 * @param 	- указатель на AXI DMA
 * @return
 * 		- ERR_DMA_UNINIT, если DMA не инициализирован
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_setKeyholeOff(axidma_t *dmainstptr) {
	if (!axidma_initialized(dmainstptr)){
		return ERR_DMA_UNINIT;
	}

	if (!axidma_tx_busy(dmainstptr)){
		uint32_t control_reg = axidma_get_tx_control(dmainstptr) & ~DMACR_KEYHOLE_MASK;
		axidma_set_tx_control(dmainstptr, control_reg);
	}

	return DMA_OK;

}

/*
 * Выключает режим KeyHole на канал приема
 * @param 	- указатель на AXI DMA
 * @return
 * 		- ERR_DMA_UNINIT, если DMA не инициализирован
 * 		- DMA_OK, если попытка отключить прошла успешно
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_setKeyholeOff(axidma_t *dmainstptr) {
	if (!axidma_initialized(dmainstptr)){
		return ERR_DMA_UNINIT;
	}

	if (!axidma_rx_busy(dmainstptr)){
		uint32_t control_reg = axidma_get_rx_control(dmainstptr) & ~DMACR_KEYHOLE_MASK;
		axidma_set_rx_control(dmainstptr, control_reg);
	}

	return DMA_OK;
}


/*
 * Возвращает состояние KeyHole канала передачи
 * @param 	- указатель на AXI DMA
 * @return
 * 		0, если KeyHole отключен
 * 		1, если KeyHole включен
 * @note
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t  	axidma_tx_isKeyholeMode	(axidma_t *dmainstptr){
	return (axidma_get_tx_control(dmainstptr) & DMACR_KEYHOLE_MASK) ? 1 : 0;
}


/*
 * Возвращает состояние KeyHole канала приема
 * @param 	- указатель на AXI DMA
 * @return
 * 		0, если KeyHole отключен
 * 		1, если KeyHole включен
 *
 * @note
 *
 * @date	- 06.04.2017
 *
 * @author	-
 * */
uint32_t  	axidma_rx_isKeyholeMode	(axidma_t *dmainstptr){
	return (axidma_get_rx_control(dmainstptr) & DMACR_KEYHOLE_MASK) ? 1 : 0;
}


/*
 * Устанавливает параметр задержки на канал передачи
 * @param 	- указатель на AXI DMA
 * @param 	- значение задержки в миллисекундах
 * @return
			- ERR_DMA_UNINIT, если AXI DMA неинициализирован
			- ERR_DMA_BAD_DELAY, если был передан недопустимый параметр задержки
			- ERR_DMA_HAD_WORK, если DMA находится в рабочем состоянии
			- DMA_OK, если задержка установлена
 *
 * @note 	при установке параметра Delay AXI DMA должен находится
 * 			в состоянии останова(Halted). Задержка(DelayFactor) должна быть в диапазоне
 * 			от 0 до 255. При delay_factor = 0 - задержка отсутствует.
 * 			Задержка позволяет ввести срабатывание прерывания для того чтобы можно было
 * 			обработать пакет(серию пакетов). Функция актуальна в том случае, когда AXI DMA
 * 			работает с параметром IRQ_THRESHOLD > 1, то есть генерируется одно прерывание на
 * 			несколько пакетов. Алгоритм работы такой: После того как придет пакет,
 * 			запускается таймер, если по достижению значения, указанного в таймере,
 * 			не придет новый пакет, то вызовется прерывание и процессор перейдет к обработке пакетов.
 * 			Таким образом данная функция позволяет не ждать пока будет передана вся серия пакетов.
 *
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_setDelay(axidma_t *dmainstptr, uint32_t delay_factor) {
	uint32_t control_reg = 0;

	if (!axidma_initialized(dmainstptr))
		return ERR_DMA_UNINIT;

	if ((delay_factor < 0) || (delay_factor > 255))
		return ERR_DMA_BAD_DELAY;

	if (axidma_tx_iswork(dmainstptr))
		return ERR_DMA_HAD_WORK;

	axidma_soft_set_tx_delay(dmainstptr, delay_factor);
	delay_factor = delay_factor << DELAY_SHIFT;
	control_reg  = (axidma_get_tx_control(dmainstptr) & ~DMACR_DELAY_MASK) | delay_factor;
	axidma_set_tx_control(dmainstptr, control_reg);

	return DMA_OK;
}


/*
 * Устанавливает параметр задержки на канал приема
 * @param 	- указатель на AXI DMA
 * @param 	- значение задержки в миллисекундах
 * @return
			- ERR_DMA_UNINIT, если AXI DMA неинициализирован
			- ERR_DMA_BAD_DELAY, если был передан недопустимый параметр задержки
			- ERR_DMA_HAD_WORK, если DMA находится в рабочем состоянии
			- DMA_OK, если задержка установлена
 *
 * @note 	при установке параметра Delay AXI DMA должен находится
 * 			в состоянии останова(Halted). Задержка(DelayFactor) должна быть в диапазоне
 * 			от 0 до 255. При delay_factor = 0 - задержка отсутствует.
 * 			Задержка позволяет ввести срабатывание прерывания для того чтобы можно было
 * 			обработать пакет(серию пакетов). Функция актуальна в том случае, когда AXI DMA
 * 			работает с параметром IRQ_THRESHOLD > 1, то есть генерируется одно прерывание на
 * 			несколько пакетов. Алгоритм работы такой: После того как придет пакет,
 * 			запускается таймер, если по достижению значения, указанного в таймере,
 * 			не придет новый пакет, то вызовется прерывание и процессор перейдет к обработке пакетов.
 * 			Таким образом данная функция позволяет не ждать пока придет вся серия пакетов.
 *
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_setDelay(axidma_t *dmainstptr, uint32_t delay_factor) {

	uint32_t control_reg = 0;

	if (!axidma_initialized(dmainstptr))
		return ERR_DMA_UNINIT;

	if ((delay_factor < 1) || (delay_factor > 255))
		return ERR_DMA_BAD_DELAY;

	if (axidma_rx_iswork(dmainstptr))
		return ERR_DMA_HAD_WORK;

	axidma_soft_set_rx_delay(dmainstptr, delay_factor);
	delay_factor = delay_factor << DELAY_SHIFT;
	control_reg  = (axidma_get_rx_control(dmainstptr) & ~DMACR_DELAY_MASK) | delay_factor;
	axidma_set_rx_control(dmainstptr, control_reg);

	return DMA_OK;

}


/*
 * Устанавливает параметр threshold на канал передачи
 * @param 	- указатель на AXI DMA
 * @param 	- количество пакетов на одно прерывание
 * @return
			- ERR_DMA_UNINIT, если AXI DMA неинициализирован
			- ERR_DMA_BAD_DELAY, если был передан недопустимый параметр threshold
			- ERR_DMA_HAD_WORK, если DMA находится в рабочем состоянии
			- DMA_OK, если threshold установлен
 *
 * @note 	Данная особенность позволяет выставлять параметр IRQ_THRESHOLD от 1 до 255.
 * 			Назначение IRQ_Threshold - организация срабатывания прерывания не по каждому
 * 			а по серии, что позволяет грузить пакеты в память серией и обрабатывать
 * 			их подобным образом.
 *
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_setThreshold	(axidma_t *dmainstptr, uint32_t threshold_factor) {
	uint32_t control_reg = 0;

	if (!axidma_initialized(dmainstptr))
		return ERR_DMA_UNINIT;

	if ((threshold_factor < 1) || (threshold_factor > 255))
		return ERR_DMA_BAD_THRESHOLD;


	if (axidma_tx_iswork(dmainstptr))
		return ERR_DMA_HAD_WORK;

	axidma_soft_set_tx_threshold(dmainstptr, threshold_factor);
	threshold_factor = threshold_factor << THRESHOLD_SHIFT;
	control_reg      = (axidma_get_tx_control(dmainstptr) & ~DMACR_THRESHOLD_MASK)
			    	 | threshold_factor;
	axidma_set_tx_control(dmainstptr, control_reg);

	return DMA_OK;
}


/*
 * Устанавливает параметр threshold на канал приема
 * @param 	- указатель на AXI DMA
 * @param 	- количество пакетов на одно прерывание
 * @return
			- ERR_DMA_UNINIT, если AXI DMA неинициализирован
			- ERR_DMA_BAD_DELAY, если был передан недопустимый параметр threshold
			- ERR_DMA_HAD_WORK, если DMA находится в рабочем состоянии
			- DMA_OK, если threshold установлен
 *
 * @note 	Данная особенность позволяет выставлять параметр IRQ_THRESHOLD от 1 до 255.
 * 			Назначение IRQ_Threshold - организация срабатывания прерывания не по каждому
 * 			а по серии, что позволяет грузить пакеты в память серией и обрабатывать
 * 			их подобным образом.
 *
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_setThreshold	(axidma_t *dmainstptr, uint32_t threshold_factor) {
	uint32_t control_reg = 0;

	if (!axidma_initialized(dmainstptr))
		return ERR_DMA_UNINIT;

	if ((threshold_factor < 1) || (threshold_factor > 255))
		return ERR_DMA_BAD_THRESHOLD;

	if (axidma_rx_iswork(dmainstptr))
		return ERR_DMA_HAD_WORK;

	axidma_soft_set_rx_threshold(dmainstptr, threshold_factor);
	threshold_factor = threshold_factor << THRESHOLD_SHIFT;
	control_reg 	 = (axidma_get_rx_control(dmainstptr) & ~DMACR_THRESHOLD_MASK)
			         | threshold_factor;
	axidma_set_rx_control(dmainstptr, control_reg);

	return DMA_OK;
}


/*
 * Возвращает параметр IRQ_DELAY с канала передачи AXI DMA
 * @param 	-	указатель axidma
 * @return
 * 		- количество задержки
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_getDelay(axidma_t *dmainstptr, uint32_t delay_factor) {
	return ((axidma_get_tx_status(dmainstptr) & DMASR_DELAY_MASK) >> DELAY_SHIFT);
}


/*
 * Возвращает параметр IRQ_DELAY с канала приема AXI DMA
 * @param 	-	указатель axidma
 * @return
 * 		- количество задержки
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t 	axidma_rx_getDelay		(axidma_t *dmainstptr, uint32_t delay_factor		){
	return ((axidma_get_rx_status(dmainstptr) & DMASR_DELAY_MASK) >> DELAY_SHIFT);
}


/*
 * Возвращает параметр IRQ_THRESHOLD с канала передачи AXI DMA
 * @param 	-	указатель axidma
 * @return
 * 		- параметр IRQ_THRESHOLD
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t 	axidma_tx_getThreshold	(axidma_t *dmainstptr, uint32_t threshold_factor	){
	return ((axidma_get_tx_status(dmainstptr) & DMASR_THRESHOLD_MASK) >> THRESHOLD_SHIFT);
}


/*
 * Возвращает параметр IRQ_THRESHOLD с канала приема AXI DMA
 * @param 	-	указатель axidma
 * @return
 * 		- параметр IRQ_THRESHOLD
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t 	axidma_rx_getThreshold	(axidma_t *dmainstptr, uint32_t threshold_factor	){
	return ((axidma_get_rx_status(dmainstptr) & DMASR_THRESHOLD_MASK) >> THRESHOLD_SHIFT);
}


/*
 * Устанавливает флаг срабатывания прерывания по завершению передачи пакета
 * на канал передачи у AXI DMA
 * @param 	-	указатель axidma
 * @return
 * 		- DMA_OK, если функция завершила свое выполнение
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_set_ioc_irq(axidma_t *dmainstptr) {
	uint32_t control_reg = axidma_get_tx_control(dmainstptr) | DMACR_IOCIRQ_MASK;

	axidma_set_tx_control(dmainstptr, control_reg);

	return DMA_OK;

}


/*
 * Устанавливает флаг срабатывания прерывания по завершению приема пакета
 * на канал приема у AXI DMA
 * @param 	-	указатель axidma
 * @return
 * 		- DMA_OK, если функция завершила свое выполнение
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_set_ioc_irq(axidma_t *dmainstptr) {
	uint32_t control_reg = axidma_get_rx_control(dmainstptr) | DMACR_IOCIRQ_MASK;
	axidma_set_rx_control(dmainstptr, control_reg);

	return DMA_OK;

}


/*
 * Устанавливает флаг срабатывания прерывания по признаку delay на
 * канал передачи у AXI DMA
 * @param 	-	указатель axidma
 * @return
 * 		- DMA_OK, если функция завершила свое выполнение
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_set_dly_irq(axidma_t *dmainstptr) {
	uint32_t control_reg = axidma_get_tx_control(dmainstptr) | DMACR_DLYIRQ_MASK;

	axidma_set_tx_control(dmainstptr, control_reg);

	return DMA_OK;
}


/*
 * Устанавливает флаг срабатывания прерывания по признаку delay на
 * канал приема у AXI DMA
 * @param 	-	указатель axidma
 * @return
 * 		- DMA_OK, если функция завершила свое выполнение
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_set_dly_irq(axidma_t *dmainstptr) {

	uint32_t control_reg = axidma_get_rx_control(dmainstptr) | DMACR_DLYIRQ_MASK;

	axidma_set_rx_control(dmainstptr, control_reg);

	return DMA_OK;

}


/*
 * Устанавливает флаг срабатывания прерывания по признаку ошибки на
 * канал передачи у AXI DMA
 * @param 	-	указатель axidma
 * @return
 * 		- DMA_OK, если функция завершила свое выполнение
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_tx_set_err_irq(axidma_t *dmainstptr) {

	uint32_t control_reg = axidma_get_tx_control(dmainstptr) | DMACR_DLYIRQ_MASK;

	axidma_set_tx_control(dmainstptr, control_reg);

	return DMA_OK;

}


/*
 * Устанавливает флаг срабатывания прерывания по признаку ошибки на
 * канал приема у AXI DMA
 * @param 	-	указатель axidma
 * @return
 * 		- DMA_OK, если функция завершила свое выполнение
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t axidma_rx_set_err_irq(axidma_t *dmainstptr) {

	uint32_t control_reg = axidma_get_rx_control(dmainstptr) | DMACR_DLYIRQ_MASK;

	axidma_set_rx_control(dmainstptr, control_reg);

	return DMA_OK;

}


/*
 * Устанавливает флаги срабатывания прерывания по завершению пакета, по delay, по ошибке
 * на канал передачи у AXI DMA
 * @param 	-	указатель axidma
 * @return
 * 		- DMA_OK, если функция завершила свое выполнение
 * @note	Сохраняет состояние
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t 	axidma_tx_enable_all_irq(axidma_t *dmainstptr){

	uint32_t control_reg = axidma_get_tx_control(dmainstptr) | DMACR_ALLIRQ_MASK;
	dmainstptr->status_tx |= (control_reg & DMACR_ALLIRQ_MASK) >> IRQ_SOFT_OFFSET;
	axidma_set_tx_control(dmainstptr, control_reg);

	return DMA_OK;

}

/*
 * Устанавливает флаги срабатывания прерывания по завершению пакета, по delay, по ошибке
 * на канал приема у AXI DMA
 * @param 	-	указатель axidma
 * @return
 * 		- DMA_OK, если функция завершила свое выполнение
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t 	axidma_rx_enable_all_irq(axidma_t *dmainstptr){
	uint32_t control_reg = axidma_get_rx_control(dmainstptr) | DMACR_ALLIRQ_MASK;
	dmainstptr->status_rx |= (control_reg & DMACR_ALLIRQ_MASK) >> IRQ_SOFT_OFFSET;
	axidma_set_rx_control(dmainstptr, control_reg);
	return DMA_OK;

}



uint32_t 	axidma_tx_disable_all_irq (axidma_t *dmainstptr){
	uint32_t control_reg = axidma_get_tx_control(dmainstptr) & ~DMACR_ALLIRQ_MASK;
	axidma_set_tx_control(dmainstptr, control_reg);
	dmainstptr->status_tx &= ~IRQ_SOFT_MASK;
	return DMA_OK;
}



uint32_t 	axidma_rx_disable_all_irq	(axidma_t *dmainstptr){
	uint32_t control_reg = axidma_get_rx_control(dmainstptr) & ~DMACR_ALLIRQ_MASK;
	dmainstptr->status_rx &= ~IRQ_SOFT_MASK;

	axidma_set_rx_control(dmainstptr, control_reg);
	return DMA_OK;
}



/*
 * Узнает наличие прерывания от канала передачи
 * @param 	-	указатель axidma
 * @return
 * 		- DMA_OK, если функция завершила свое выполнение
 * @note	После этого обычно надо узнать характер прерывания
 * 			для того, чтобы выбрать необходимую последовательность действий.
 * 			А дальше надо вызвать метод axidma_tx_ack_irq, чтобы ответить AXI DMA
 * 			о том, что процессор знает о наличии прерывания
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t 	axidma_tx_get_irq		(axidma_t *dmainstptr){
	return (axidma_get_tx_status(dmainstptr) & DMASR_IRQALL_MASK);
}


/*
 * Узнает наличие прерывания от канала приема
 * @param 	-	указатель axidma
 * @return
 * 		- DMA_OK, если функция завершила свое выполнение
 * @note	После этого обычно надо узнать характер прерывания
 * 			для того, чтобы выбрать необходимую последовательность действий.
 * 			А дальше надо вызвать метод axidma_rx_ack_irq, чтобы ответить AXI DMA
 * 			о том, что процессор знает о наличии прерывания
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t 	axidma_rx_get_irq		(axidma_t *dmainstptr){
	return (axidma_get_rx_status(dmainstptr) & DMASR_IRQALL_MASK);
}


/*
 * Посылает ответ для AXI DMA для того чтобы очистить флаг прерывания AXI DMA
 * у канала передачи данных
 * @param 	-	указатель axidma
 * @return
 * 		- DMA_OK, если функция завершила свое выполнение
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t 	axidma_tx_ack_irq		(axidma_t *dmainstptr, uint32_t irq) {
	uint32_t status_reg = axidma_get_tx_status(dmainstptr) & DMASR_IRQALL_MASK;

	axidma_set_tx_status(dmainstptr, status_reg);
	if (status_reg & DMASR_ERRIRQ_MASK){
		return irq;
	}

	return DMA_OK;

}


/*
 * Посылает ответ для AXI DMA для того чтобы очистить флаг прерывания AXI DMA
 * у канала приема данных
 * @param 	-	указатель axidma
 * @return
 * 		- DMA_OK, если функция завершила свое выполнение
 * @note	NONE
 * @date	- 06.04.2017
 * @author	-
 * */
uint32_t 	axidma_rx_ack_irq (axidma_t *dmainstptr, uint32_t irq) {
	int status = 0;
	uint32_t status_reg = axidma_get_rx_status(dmainstptr);
	status_reg &= DMASR_IRQALL_MASK;

	status = axidma_set_rx_status(dmainstptr, status_reg);
	if (status != DMA_OK){
		return -1;
	}

	if (status_reg & DMASR_ERRIRQ_MASK){
		return irq;
	}


	return DMA_OK;

}


/*
 * Возвращает указатель на цепочку дескрипторов axidma
 * @param 	- указатель на AXI DMA
 * @return
 * 		- указатель на начало цепочки дескрипторов канала приема данных
 * @note
 * @date	-	07.04.2017
 * @author	-
 * */
bdring_t *axidma_get_rxring	(axidma_t *dmainstptr) {
	return (&((dmainstptr)->rxring));
}


/*
 * Возвращает указатель на цепочку дескрипторов axidma
 * @param 	- указатель на AXI DMA
 * @return
 * 		- указатель на начало цепочки дескрипторов канала передачи данных
 * @note
 * @date	-	07.04.2017
 * @author	-
 * */
bdring_t *axidma_get_txring		(axidma_t *dmainstptr){
	return (&((dmainstptr)->txring));
}


uint32_t axidma_api_run(axidma_t *dmainstptr, uint32_t axidma_hw_address){
	int status = 0;
	status = axidma_initialization(dmainstptr, axidma_hw_address);
	if (status != DMA_OK){
		return status;
	}

	status = axidma_api_reset(dmainstptr);
	if (status != DMA_OK){
		return status;
	}

	if (axidma_api_hasrx(dmainstptr)) {
		bdring_t *rxring = axidma_get_rxring(dmainstptr);
		axidma_rx_disable_all_irq(dmainstptr);

		status = bdring_initialize(rxring, RX_BASEADDR, RX_BUFFER_BASE,
				                   RX_BUFFER_SIZE, INIT_RXCHAN_CFG);
		if (status != RING_OK){
			return status;
		}

		axidma_rx_enable_all_irq(dmainstptr);
		status = axidma_rx_setDelay(dmainstptr, 0/*RX_DELAY*/);
		if (status != DMA_OK){
			return status;
		}

		status = axidma_rx_setThreshold(dmainstptr, 1/*RX_THRESHOLD*/);
		if (status != DMA_OK){
			return status;
		}
	}

	if (axidma_api_hastx(dmainstptr)) {
		bdring_t *txring = axidma_get_txring(dmainstptr);
		axidma_tx_disable_all_irq(dmainstptr);

		status = bdring_initialize(txring, TX_BASEADDR, TX_BUFFER_BASE,
				                   TX_BUFFER_SIZE, INIT_TXCHAN_CFG);
		if (status != RING_OK){
			return status;
		}

		axidma_tx_enable_all_irq(dmainstptr);
		status = axidma_tx_setDelay(dmainstptr, 0/*TX_DELAY*/);
		if (status != DMA_OK){
			return status;
		}
		status = axidma_tx_setThreshold(dmainstptr, 1/*TX_THRESHOLD*/);
		if (status != DMA_OK){
			return status;
		}
	}
/*Тут запустится только канал приема*/
	status = axidma_api_start(dmainstptr);
	if (status != DMA_OK) {
		return status;
	}
	return DMA_OK;
}


uint32_t axidma_api_hasrx(axidma_t *dmainstptr) {
	return dmainstptr->isRx;
}


uint32_t axidma_api_hastx(axidma_t *dmainstptr) {
	return dmainstptr->isTx;
}


/**
 *
 */
uint32_t axidma_api_send(axidma_t *dmainstptr, memory_buffer_t *membufinstptr,
		                 uint32_t size) {
	int status        = 0;
	int involved_bds  = api_get_bdcnt(BYTES_PER_DESC, size);
	int involved_size = size;
	int index         = 0;

	bdring_t *txring = axidma_get_txring(dmainstptr);
	bd_t* current_bd = (bd_t*)bdring_get_head(txring);
	for (index = 0; index < (involved_bds-1); index++) {
		if (index == 0) {
			bd_set_sof(current_bd);
		}
		bd_set_length(current_bd, BYTES_PER_DESC);
		involved_size -= BYTES_PER_DESC;
		current_bd     = (bd_t*)bdring_next_bd(current_bd);
	}

	bd_set_nextdesc(current_bd, txring->bd_headaddr);
	if (involved_bds == 1)
		bd_set_sof(current_bd);
	bd_set_eof	   (current_bd);
	bd_set_length  (current_bd, involved_size);

	uint32_t tail_register = api_get_tailmem_addr(txring->bd_headaddr,
			                                      involved_bds);
	usleep(500);
	// Start sending data
	axidma_set_tx_curdesc(dmainstptr, txring->bd_headaddr);
	status = axidma_tx_start(dmainstptr);
	if (status != DMA_OK) {
		return -status;
	}

	int delay_cnt = RESET_TIMEOUT;
	while (!axidma_tx_iswork(dmainstptr)) {
		delay_cnt--;
		if (delay_cnt == 0){
			return -ERR_DMA_HAD_WORK;
		}
	}
	axidma_set_tx_taildesc(dmainstptr, tail_register);

	int tx_irq = 0;
	size_t timeout_delay = 20048576;
	while(!tx_irq && (timeout_delay != 0)) {
		tx_irq = axidma_tx_get_irq(dmainstptr);
		timeout_delay--;
	}

	status = axidma_tx_ack_irq(dmainstptr, tx_irq);
	if (status != DMA_OK){
		printf("tx_irq_err: %x\r\n", status);
		return -1;
	}

	int transferred_bytes = axidma_api_process_tx(dmainstptr);

	status = axidma_tx_reset(dmainstptr);
	if (status != DMA_OK)
		return status;

	int delay_tx = axidma_soft_get_tx_delay 	(dmainstptr);
	int thres_tx = axidma_soft_get_tx_threshold	(dmainstptr);

	axidma_tx_setDelay 		(dmainstptr, delay_tx);
	axidma_tx_setThreshold 	(dmainstptr, thres_tx);
	axidma_tx_enable_all_irq(dmainstptr);

	return transferred_bytes;
}


uint32_t axidma_api_receive(axidma_t *dmainstptr, memory_buffer_t *membufinstptr){
	int status = 0;

    size_t timeout_delay = 20048576;
	int rx_irq = axidma_rx_get_irq(dmainstptr);
	while(!rx_irq && (timeout_delay != 0)) { // XXX:
		rx_irq = axidma_rx_get_irq(dmainstptr);
		timeout_delay--;
	}

	status = axidma_rx_ack_irq(dmainstptr, rx_irq);
	if (status != DMA_OK) {
		printf("rx_irq_err 0x%08x\r\n", status);
		return status;
	}
	int size = axidma_api_process_rx(dmainstptr);

	status = axidma_rx_pause(dmainstptr);
	if (status != DMA_OK) {
		printf("ERR1: 0x%08x", status);
	}

	status = axidma_api_rx_reload(dmainstptr);
	if (status != DMA_OK) {
		printf("ERR2: 0x%08x", status);
	}
	return size;
}


#define RECV_DBG			1
uint32_t axidma_api_process_tx(axidma_t *dmainstptr){
	bdring_t *txring = axidma_get_txring(dmainstptr);
	int proc_bds 	 = bdring_get_processed_bds(txring);
	int proc_bds_cnt = 0;

	bd_t *current_bd = (bd_t*)bdring_get_head(txring);
	int size         = 0;
	for (proc_bds_cnt = 0; proc_bds_cnt < proc_bds; proc_bds_cnt++) {
		if (bd_isSof(current_bd))
			size  = bd_get_transferred_len(current_bd);
		else if (bd_isIof(current_bd))
			size += bd_get_transferred_len(current_bd);
		else if ((bd_isEof(current_bd)) && (!bd_isSof(current_bd)))
			size += bd_get_transferred_len(current_bd);

		bd_free(current_bd, TX_CHANNEL);
		current_bd = (bd_t*)bdring_next_bd(current_bd);
	}

	current_bd = (bd_t*)bdring_get_head(txring);

	return size;
}


// TODO: rewrite if
uint32_t axidma_api_process_rx(axidma_t *dmainstptr) {
	bdring_t *rxring   = axidma_get_rxring(dmainstptr);
	int proc_bds     = bdring_get_processed_bds(rxring);
	int proc_bds_cnt = 0;
	bd_t *current_bd   = (bd_t*)bdring_get_head(rxring);
	int size         = 0;
	for (proc_bds_cnt = 0; proc_bds_cnt < proc_bds; proc_bds_cnt++) {
		if (bd_isSof(current_bd)) {
			size = bd_get_transferred_len(current_bd);
		}
		if (bd_isIof(current_bd)) {
			size += bd_get_transferred_len(current_bd);
		}
		if ((bd_isEof(current_bd)) && (!bd_isSof(current_bd))) {
			size += bd_get_transferred_len(current_bd);
		}
		/*if (bd_isEof(current_bd)){
			//
		}*/

		bd_free(current_bd, RX_CHANNEL);

		current_bd = (bd_t*)bdring_next_bd(current_bd);
	}

	return size;
}


uint32_t axidma_api_rx_reload(axidma_t *dmainstptr) {
	int status = 0;

	status = axidma_rx_reset(dmainstptr);
	if (status != DMA_OK){
		return -1;
	}

	int delay_rx = axidma_soft_get_rx_delay	   (dmainstptr);
	int thres_rx = axidma_soft_get_rx_threshold(dmainstptr);

	axidma_rx_setDelay 		(dmainstptr, delay_rx);
	axidma_rx_setThreshold	(dmainstptr, thres_rx);
	axidma_rx_enable_all_irq(dmainstptr);


	uint32_t head_desc = dmainstptr->rxring.bd_headaddr;
	uint32_t tail_desc = dmainstptr->rxring.bd_lastaddr;

	axidma_set_rx_curdesc(dmainstptr, head_desc);
	status = axidma_rx_start(dmainstptr);
	if (status != DMA_OK){
		return -1;
	}

	int delay_cnt = RESET_TIMEOUT;
	while(!axidma_rx_iswork(dmainstptr)){
		delay_cnt--;
		if (delay_cnt == 0){
			return ERR_DMA_HAD_WORK;
		}
	}
	axidma_set_rx_taildesc(dmainstptr, tail_desc);

	return DMA_OK;
}
