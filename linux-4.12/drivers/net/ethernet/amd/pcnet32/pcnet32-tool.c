#include "pcnet32.h"

u16 pcnet32_wio_read_csr(unsigned long addr, int index) {
	outw(index, addr + PCNET32_WIO_RAP);
	return inw(addr + PCNET32_WIO_RDP);
}

static void pcnet32_wio_write_csr(unsigned long addr, int index, u16 val) {
	outw(index, addr + PCNET32_WIO_RAP);
	outw(val, addr + PCNET32_WIO_RDP);
}

static u16 pcnet32_wio_read_bcr(unsigned long addr, int index) {
	outw(index, addr + PCNET32_WIO_RAP);
	return inw(addr + PCNET32_WIO_BDP);
}

static void pcnet32_wio_write_bcr(unsigned long addr, int index, u16 val) {
	outw(index, addr + PCNET32_WIO_RAP);
	outw(val, addr + PCNET32_WIO_BDP);
}

static u16 pcnet32_wio_read_rap(unsigned long addr) {
	return inw(addr + PCNET32_WIO_RAP);
}

static void pcnet32_wio_write_rap(unsigned long addr, u16 val) {
	outw(val, addr + PCNET32_WIO_RAP);
}

void pcnet32_wio_reset(unsigned long addr) {
	inw(addr + PCNET32_WIO_RESET);
}

int pcnet32_wio_check(unsigned long addr) {
	outw(88, addr + PCNET32_WIO_RAP);
	return inw(addr + PCNET32_WIO_RAP) == 88;
}

const struct pcnet32_access pcnet32_wio =
{ //
	.read_csr = pcnet32_wio_read_csr, //
	.write_csr = pcnet32_wio_write_csr,
	.read_bcr = pcnet32_wio_read_bcr,
	.write_bcr = pcnet32_wio_write_bcr,
	.read_rap = pcnet32_wio_read_rap,
	.write_rap = pcnet32_wio_write_rap,
	.reset = pcnet32_wio_reset };

u16 pcnet32_dwio_read_csr(unsigned long addr, int index) {
	outl(index, addr + PCNET32_DWIO_RAP);
	return inl(addr + PCNET32_DWIO_RDP) & 0xffff;
}

static void pcnet32_dwio_write_csr(unsigned long addr, int index, u16 val) {
	outl(index, addr + PCNET32_DWIO_RAP);
	outl(val, addr + PCNET32_DWIO_RDP);
}

static u16 pcnet32_dwio_read_bcr(unsigned long addr, int index) {
	outl(index, addr + PCNET32_DWIO_RAP);
	return inl(addr + PCNET32_DWIO_BDP) & 0xffff;
}

static void pcnet32_dwio_write_bcr(unsigned long addr, int index, u16 val) {
	outl(index, addr + PCNET32_DWIO_RAP);
	outl(val, addr + PCNET32_DWIO_BDP);
}

static u16 pcnet32_dwio_read_rap(unsigned long addr) {
	return inl(addr + PCNET32_DWIO_RAP) & 0xffff;
}

static void pcnet32_dwio_write_rap(unsigned long addr, u16 val) {
	outl(val, addr + PCNET32_DWIO_RAP);
}

void pcnet32_dwio_reset(unsigned long addr) {
	inl(addr + PCNET32_DWIO_RESET);
}

int pcnet32_dwio_check(unsigned long addr) {
	outl(88, addr + PCNET32_DWIO_RAP);
	return (inl(addr + PCNET32_DWIO_RAP) & 0xffff) == 88;
}

const struct pcnet32_access pcnet32_dwio =
{ //
	.read_csr = pcnet32_dwio_read_csr, //
	.write_csr = pcnet32_dwio_write_csr,
	.read_bcr = pcnet32_dwio_read_bcr,
	.write_bcr = pcnet32_dwio_write_bcr,
	.read_rap = pcnet32_dwio_read_rap,
	.write_rap = pcnet32_dwio_write_rap,
	.reset = pcnet32_dwio_reset };
