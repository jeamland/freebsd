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

#define	MSGBUS_GROUP_NEW	0

struct mbgroupreq {
	uint64_t	mbgr_id;
	struct		sockaddr_msgbus mbgr_addr;
};

#ifdef _KERNEL

struct msgbus_groupentry {
	STAILQ_ENTRY(msgbus_groupentry)	mge_link;
	struct	sockaddr_msgbus mge_addr;
};

struct msgbus_group {
	LIST_ENTRY(msgbus_group)		mg_link;
	uint64_t				mg_id;
	STAILQ_HEAD(, msgbus_groupentry)	mg_entries;
};

struct msgbuspcb {
	LIST_ENTRY(msgbuspcb) mbp_link;
	struct		socket *mbp_socket;
	struct		sockaddr_msgbus mbp_name;
	LIST_HEAD(, msgbus_group) mbp_groups;
	struct		mtx mbp_mtx;

#define	mbp_address	mbp_name.smb_address
};

#define	sotomsgbuspcb(so)	((struct msgbuspcb *)((so)->so_pcb))

#endif /* _KERNEL */

#endif /* _SYS_MSGBUS_H_ */
