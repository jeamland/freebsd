#ifndef _SYS_MSGBUS_H_
#define	_SYS_MSGBUS_H_

struct msgbuspcb {
	struct	socket *mbp_socket;
	struct	mtx mbp_mtx;
};

#define	sotomsgbuspcb(so)	((struct msgbuspcb *)((so)->so_pcb))

#endif /* _SYS_MSGBUS_H_ */
