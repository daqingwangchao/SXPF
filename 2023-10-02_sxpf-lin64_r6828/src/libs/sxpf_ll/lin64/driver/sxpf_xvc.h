#ifndef SXPF_XVC_H_
#define SXPF_XVC_H_


// forward decl from sxpf_module.h
typedef struct sxpf_pci_device_s    sxpf_card_t;


int xil_xvc_init(void);
void xil_xvc_exit(void);
void xil_probe_xvc(sxpf_card_t *card);
long ioctl_xvc(sxpf_card_t* card, unsigned int cmd, unsigned long arg);


#endif /* defined(SXPF_XVC_H_) */
