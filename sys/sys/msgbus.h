#ifndef _SYS_MSGBUS_H_
#define	_SYS_MSGBUS_H_

#include <sys/cdefs.h>
#include <sys/_types.h>

#ifndef _SA_FAMILY_T_DECLARED
typedef	__sa_family_t	sa_family_t;
#define	_SA_FAMILY_T_DECLARED
#endif

/*
 * Definitions for message bus IPC domain.
 */
struct sockaddr_msgbus {
	unsigned char	smb_len;	/* sockaddr len including null */
	sa_family_t	smb_family;	/* AF_MSGBUS */
	uint64_t	smb_address;	/* unique address */
};

#ifdef _KERNEL

struct msgbuspcb {
	struct		socket *mbp_socket;
	uint64_t	mbp_address;
	struct		mtx mbp_mtx;
};

#define	sotomsgbuspcb(so)	((struct msgbuspcb *)((so)->so_pcb))

#endif /* _KERNEL */

#endif /* _SYS_MSGBUS_H_ */
