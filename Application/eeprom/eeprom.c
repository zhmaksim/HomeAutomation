/*
 * Copyright (C) 2024 zhmaksim <zhiharev.maxim.alexandrovich@yandex.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/* Includes ---------------------------------------------------------------- */

#include "eeprom.h"
#include "eeprom_data.h"
#include "crc32.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define EEPROM_PAGE_SIZE                0x40
#define EEPROM_UID_MEM_ADDRESS          0x0000
#define EEPROM_CACHE_MEM_ADDRESS        0x7E00
#define EEPROM_CACHE_SIZE               0x200

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик I2C */
extern struct i2c_handle i2c1;

/* Mutex и Event Group I2C */
extern SemaphoreHandle_t i2c1_mutex;
extern EventGroupHandle_t i2c1_event_group;

/* Обработчик EEPROM */
struct eeprom_handle eeprom = {
    .i2c = &i2c1,
    .slave_address = 0xA0,
    .uid = {
        .version = 1,
        .data = {0x1A, 0x4A, 0x33, 0xB7, 0x20, 0x2A, 0xE7, 0xF9},
    },
};

/* Private function prototypes --------------------------------------------- */

static void eeprom_process(void *arg);

static uint8_t eeprom_verify_uid(struct eeprom_handle *handle);

static void eeprom_load_data(struct eeprom_handle *handle);

static void eeprom_upload_data(struct eeprom_handle *handle);

static hal_status_t eeprom_write_data(struct eeprom_handle *handle, struct eeprom_data* data_item);

static hal_status_t eeprom_read_data(struct eeprom_handle *handle, struct eeprom_data* data_item);

static hal_status_t eeprom_mem_write(struct eeprom_handle *handle, uint16_t mem_address, const void *data, size_t size);

static hal_status_t eeprom_mem_read(struct eeprom_handle *handle, uint16_t mem_address, void* data, size_t size);

static uint32_t eeprom_crc(const void *data, size_t size);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 */
void eeprom_init(struct eeprom_handle *handle)
{
    assert(handle != NULL);
    assert(handle->i2c != NULL);

    /* Установить Mutex и Event Group */
    if (handle->i2c == &i2c1) {
        handle->i2c_mutex = i2c1_mutex;
        handle->i2c_event_group = i2c1_event_group;
    }

    /* Создать очередь */
    handle->queue = xQueueCreate(EEPROM_QUEUE_SIZE, sizeof(uint32_t));
    if (handle->queue == NULL)
        hal_error();

    /* Проверить готовность устройства */
    if (!hal_i2c_device_is_ready(handle->i2c, handle->slave_address, 3, 10))
        hal_error();

    xTaskCreate(eeprom_process,
                "eeprom",
                configMINIMAL_STACK_SIZE * 2,
                (void *) handle,
                tskIDLE_PRIORITY + 2,
                NULL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать завершение команды I2C EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 */
void eeprom_i2c_command_completed_it_handler(struct eeprom_handle *handle)
{
    assert(handle != NULL);
    assert(handle->i2c_event_group != NULL);

    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xEventGroupSetBitsFromISR(handle->i2c_event_group,
                                  I2C_EV_CMD_CPLT,
                                  &higher_priority_task_woken) == pdPASS) {
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать ошибку I2C EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 */
void eeprom_i2c_error_it_handler(struct eeprom_handle *handle)
{
    assert(handle != NULL);
    assert(handle->i2c_event_group != NULL);

    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xEventGroupSetBitsFromISR(handle->i2c_event_group,
                                  I2C_EV_ERR,
                                  &higher_priority_task_woken) != pdFAIL) {
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Загрузить EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 */
void eeprom_load(struct eeprom_handle *handle)
{
    assert(handle != NULL);

    /*
     * Если уникальный идентификатор корректный - загружаем данные из памяти в устройство
     * иначе выгружаем данные из устройства
     */
    if (eeprom_verify_uid(handle) == HAL_OK) {
        eeprom_load_data(handle);
    } else {
        eeprom_upload_data(handle);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Проверить уникальные данные EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 * @return          Статус @ref hal_status_t
 */
static hal_status_t eeprom_verify_uid(struct eeprom_handle *handle)
{
    struct eeprom_uid uid;

    if (eeprom_mem_read(handle,
                        EEPROM_UID_MEM_ADDRESS,
                        (void *) &uid,
                        sizeof(struct eeprom_uid)) != HAL_OK) {
        handle->state = EEPROM_READ_ERROR;
        return HAL_ERROR;
    }

    if (handle->uid.version != uid.version ||
        memcmp(handle->uid.data, uid.data, sizeof(handle->uid.data)) != 0) {
        if (eeprom_mem_write(handle,
                             EEPROM_UID_MEM_ADDRESS,
                             (void *) &handle->uid,
                             sizeof(struct eeprom_uid)) != HAL_OK) {
            handle->state = EEPROM_WRITE_ERROR;
        }
        return HAL_ERROR;
    }

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Загрузить данные EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 */
static void eeprom_load_data(struct eeprom_handle *handle)
{
    for (register uint32_t index = 0; index < UINT32_MAX; index++) {
        struct eeprom_data *data_item = eeprom_data_by_index(index);
        if (data_item == NULL)
            break;

        eeprom_read_data(handle, data_item);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выгрузить данные EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 */
static void eeprom_upload_data(struct eeprom_handle *handle)
{
    for (register uint32_t index = 0; index < UINT32_MAX; index++) {
        struct eeprom_data *data_item = eeprom_data_by_index(index);
        if (data_item == NULL)
            break;

        eeprom_write_data(handle, data_item);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Записать данные EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 * @param[in]       data: Указатель на данные
 * @return          Статус @ref hal_status_t
 */
hal_status_t eeprom_write(struct eeprom_handle *handle, const void *data)
{
    assert(handle != NULL);
    assert(handle->queue != NULL);

    if (data == NULL)
        return HAL_ERROR;

    uint32_t address_data_item = (uint32_t) data;

    if (xQueueSend(handle->queue, &address_data_item, pdMS_TO_TICKS(300)) != pdPASS)
        return HAL_ERROR;

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Проверить работоспособность EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 * @return          Состояние работоспособности
 */
bool eeprom_is_work(struct eeprom_handle *handle)
{
    return handle->state == EEPROM_WORK ? true : false;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать события EEPROM
 *
 * @param[in]       arg: Указатель на параметры
 */
static void eeprom_process(void *arg)
{
    static const TickType_t frequency = pdMS_TO_TICKS(25);

    struct eeprom_handle *handle = arg;

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake_time, frequency);

        uint32_t address_data_item;

        if (xQueueReceive(handle->queue, &address_data_item, portMAX_DELAY) != pdPASS)
            continue;

        if (eeprom_is_work(handle)) {
            void *data = (void *) address_data_item;

            struct eeprom_data *data_item = eeprom_data(data);
            if (data_item != NULL)
                eeprom_write_data(handle, data_item);
        }
    }

    vTaskDelete(NULL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Записать элемент данных EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 * @param[in]       data_item: Указатель на структуру элемента данных EEPROM
 * @return          Статус @ref hal_status_t
 */
static hal_status_t eeprom_write_data(struct eeprom_handle *handle, struct eeprom_data *data_item)
{
    if (!eeprom_is_work(handle))
        return HAL_ERROR;

    struct eeprom_data_header *data_header = NULL;
    uint32_t data_header_size = sizeof(struct eeprom_data_header);
    uint32_t data_size = data_header_size + data_item->size;

    void *data = pvPortMalloc(data_size);
    if (data == NULL) {
        handle->state = EEPROM_MALLOC_ERROR;
        return HAL_ERROR;
    }

    data_header = data;

    data_header->id = data_item->id;
    data_header->data_size = data_item->size;
    data_header->crc = eeprom_crc(data_item->data, data_item->size);

    taskENTER_CRITICAL();
    {
        memcpy(data + data_header_size,
               data_item->data,
               data_item->size);
    }
    taskEXIT_CRITICAL();

    if (eeprom_mem_write(handle, EEPROM_CACHE_MEM_ADDRESS, data, data_size) != HAL_OK) {
        vPortFree(data);

        handle->state = EEPROM_WRITE_ERROR;
        return HAL_ERROR;
    }

    if (eeprom_mem_write(handle, data_item->mem_address, data, data_size) != HAL_OK) {
        vPortFree(data);

        handle->state = EEPROM_WRITE_ERROR;
        return HAL_ERROR;
    }

    vPortFree(data);

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прочитать элемент данных EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 * @param[in]       data_item: Указатель на структуру элемента данных EEPROM
 * @return          Статус @ref hal_status_t
 */
static hal_status_t eeprom_read_data(struct eeprom_handle *handle, struct eeprom_data *data_item)
{
    if (!eeprom_is_work(handle))
        return HAL_ERROR;

    struct eeprom_data_header *data_header = NULL;
    uint32_t data_header_size = sizeof(struct eeprom_data_header);
    uint32_t data_size = data_header_size + data_item->size;

    void *data = pvPortMalloc(data_size);
    if (data == NULL) {
        handle->state = EEPROM_MALLOC_ERROR;
        return HAL_ERROR;
    }

    if (eeprom_mem_read(handle, data_item->mem_address, data, data_size) != HAL_OK) {
        vPortFree(data);

        handle->state = EEPROM_READ_ERROR;
        return HAL_ERROR;
    }

    data_header = data;

    if (data_item->id == data_header->id &&
        data_item->size == data_header->data_size) {
        uint32_t crc = eeprom_crc(data + data_header_size, data_item->size);

        if (crc == data_header->crc) {
            taskENTER_CRITICAL();
            {
                memcpy(data_item->data,
                       data + data_header_size,
                       data_item->size);
            }
            taskEXIT_CRITICAL();

            vPortFree(data);

            return HAL_OK;
        }
    }

    if (eeprom_mem_read(handle, EEPROM_CACHE_MEM_ADDRESS, data, data_size) != HAL_OK) {
        vPortFree(data);

        handle->state = EEPROM_READ_ERROR;
        return HAL_ERROR;
    }

    data_header = data;

    if (data_item->id == data_header->id &&
        data_item->size == data_header->data_size) {
        uint32_t crc = eeprom_crc(data + data_header_size, data_item->size);

        if (crc == data_header->crc) {
            taskENTER_CRITICAL();
            {
                memcpy(data_item->data,
                       data + data_header_size,
                       data_item->size);
            }
            taskEXIT_CRITICAL();

            vPortFree(data);

            return HAL_OK;
        }
    }

    vPortFree(data);

    return HAL_ERROR;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Записать данные в память EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 * @param[in]       mem_address: Адрес памяти
 * @param[in]       data: Указатель на данные
 * @param[in]       size: Размер данных
 * @return          Статус @ref hal_status_t
 */
static hal_status_t eeprom_mem_write(struct eeprom_handle *handle, uint16_t mem_address, const void *data, size_t size)
{
    hal_status_t status = HAL_ERROR;

    if (data == NULL || size == 0)
        return status;

    if (xSemaphoreTake(handle->i2c_mutex, pdMS_TO_TICKS(50)) == pdPASS) {
        uint16_t page_end;
        uint16_t bw;

        while (size > 0) {
            page_end = (mem_address & 0xFFC0) + EEPROM_PAGE_SIZE;

            if (size > page_end - mem_address) {
               bw = page_end - mem_address;
            } else {
               bw = size;
            }

            /* Проверить готовность устройства */
            while (!hal_i2c_device_is_ready(handle->i2c, handle->slave_address, 1, 10))
                vTaskDelay(pdMS_TO_TICKS(3));

            /* Ожидание готовности */
            hal_i2c_wait_ready(handle->i2c, HAL_MAX_DELAY);

            /* Запись данные */
            hal_i2c_mem_write_it(handle->i2c,
                                 handle->slave_address,
                                 mem_address,
                                 sizeof(mem_address),
                                 data,
                                 bw);

            /* Ожидание завершения записи данных */
            EventBits_t bits = xEventGroupWaitBits(handle->i2c_event_group,
                                                   I2C_EV_CMD_CPLT | I2C_EV_ERR,
                                                   pdTRUE,
                                                   pdFALSE,
                                                   portMAX_DELAY);
            if (READ_BIT(bits, I2C_EV_CMD_CPLT_Msk)) {
                status = HAL_OK;
            } else {
                status = HAL_ERROR;
                break;
            }

            mem_address += bw;
            data += bw;
            size -= bw;
        }

        xSemaphoreGive(handle->i2c_mutex);
    }

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прочитать данные из памяти EEPROM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика EEPROM
 * @param[in]       mem_address: Адрес памяти
 * @param[out]      data: Указатель на данные
 * @param[in]       size: Размер данных
 * @return          Статус @ref hal_status_t
 */
static hal_status_t eeprom_mem_read(struct eeprom_handle *handle, uint16_t mem_address, void *data, size_t size)
{
    hal_status_t status = HAL_ERROR;

    if (data == NULL || size == 0)
        return status;

    if (xSemaphoreTake(handle->i2c_mutex, pdMS_TO_TICKS(50)) == pdPASS) {
        /* Проверить готовность устройства */
        while (!hal_i2c_device_is_ready(handle->i2c, handle->slave_address, 1, 10))
            vTaskDelay(pdMS_TO_TICKS(3));

        /* Ожидание готовности */
        hal_i2c_wait_ready(handle->i2c, HAL_MAX_DELAY);

        /* Прочитать данные */
        hal_i2c_mem_read_it(handle->i2c,
                            handle->slave_address,
                            mem_address,
                            sizeof(mem_address),
                            data,
                            size);

        /* Ожидание завершения чтения данных */
        EventBits_t bits = xEventGroupWaitBits(handle->i2c_event_group,
                                               I2C_EV_CMD_CPLT | I2C_EV_ERR,
                                               pdTRUE,
                                               pdFALSE,
                                               portMAX_DELAY);
        if (READ_BIT(bits, I2C_EV_CMD_CPLT_Msk)) {
            status = HAL_OK;
        }

        xSemaphoreGive(handle->i2c_mutex);
    }

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief       Контрольная сумма EEPROM
 *
 * @param[in]   data: Указатель на данные
 * @param[in]   size: Размер данных
 * @retrun      Значение контрольной суммы CRC32
 */
static uint32_t eeprom_crc(const void *data, size_t size)
{
    return crc32(data, size);
}
/* ------------------------------------------------------------------------- */
