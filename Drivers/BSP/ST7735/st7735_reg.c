#include "st7735_reg.h"

int32_t st7735_read_reg(st7735_ctx_t *ctx, uint8_t reg, uint8_t *pdata) {
	return ctx->ReadReg(ctx->handle, reg, pdata);
}

int32_t st7735_write_reg(st7735_ctx_t *ctx, uint8_t reg, uint8_t *pdata, uint32_t length) {
	return ctx->WriteReg(ctx->handle, reg, pdata, length);
}


int32_t st7735_send_data(st7735_ctx_t *ctx, uint8_t *pdata, uint32_t length) {
	return ctx->SendData(ctx->handle, pdata, length);
}


int32_t st7735_recv_data(st7735_ctx_t *ctx, uint8_t *pdata, uint32_t length) {
	return ctx->RecvData(ctx->handle, pdata, length);
}
