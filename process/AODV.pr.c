/* Process model C form file: AODV.pr.c */
/* Portions of this file copyright 1986-2008 by OPNET Technologies, Inc. */



/* This variable carries the header into the object file */
const char AODV_pr_c [] = "MIL_3_Tfile_Hdr_ 145A 30A modeler 7 4CCF9A08 4CCF9A08 1 china-0f9728557 Administrator 0 0 none none 0 0 none 0 0 0 0 0 0 0 0 1e80 8                                                                                                                                                                                                                                                                                                                                                                                            ";
#include <string.h>



/* OPNET system definitions */
#include <opnet.h>



/* Header Block */

//Define packet type
#define rreq_pk		1
#define rrep_pk		2
#define rerr_pk		3
#define data_pk		4

//Define stream in-out NO.
#define SRC_STRM 		1
#define RCV_STRM 		0
#define SEND_STRM 		0
#define DISCARD_STRM 	1
#define BROADCAST   	-1

//Define queue type
#define data_queue		0
#define grade_queue		1

/* Maximum number of nodes in the net */
#define N 100

/* infinity */
#define INFINITY -1
/* routing entry state */
#define NOT_VALID -1
#define NON_EXISTENT -2

/* Remote intrpt or self intrpt codes */
#define ACK_CODE      10000
#define NACK_CODE     20000
#define REP_CODE      30000//self intrp code in order to serve buffer
#define NREPLY_CODE   40000
/* Offset */
#define OPTION 10000

//Define self-interrupt code and transition condition
#define END	        		    (op_intrpt_type() == OPC_INTRPT_ENDSIM)
#define FROM_LOWER				((op_intrpt_type() == OPC_INTRPT_STRM) && (op_intrpt_strm() == RCV_STRM))
#define FROM_UPPER				((op_intrpt_type() == OPC_INTRPT_STRM) && (op_intrpt_strm() == SRC_STRM))

#define NO_REPLY               (op_intrpt_type() == OPC_INTRPT_REMOTE && ((int)(op_intrpt_code()/OPTION))*OPTION == NREPLY_CODE)
#define Route_Expiration       (op_intrpt_type() == OPC_INTRPT_SELF && (op_intrpt_code() < N) )

/* request sent status */
typedef enum
{
	OFF = 0, // Request was not sent
	WAITING_FOR_REPLY = 1,   // Request waiting for Reply
	WAITING_FOR_REPAIR = 2   // Request for repair
}Request_Status_Type;

typedef struct
{
	int sequence_number;
	int nbOfRetries;
	double expirationTime;
	Request_Status_Type status;
	int ttl;
	//int gratuitous;
	Evhandle evt;
}RequestSentAttributes;

typedef struct 
{
	int broadcastID;
	double expirationTime;
} RequestSeenAttributes; 

typedef enum
{
	ACTIVE = 0,       // Entry is active
  	INVALID = 1,      // Entry is invalid
	UNDER_REPAIR = 2  // Entry is under repair. It must not be deleted.
}Route_Status_Type;

typedef struct
{
	int dest;
	int destSeqNo;
	int hopCount;
	int lastHopCount;
	int nextHop;
	List* listOfPrecursors;
	double expirationTime;
	double lastExpirationTime;
	Route_Status_Type status;
	Evhandle expiration_evt;
}RoutingTableEntry;

typedef struct
{
	Evhandle evt;
	Packet* copy_pk_ptr;
	double schedule;
} AckEvt;

typedef struct
{
	AckEvt* ackEvt;
	int		listnumber;
}AckEvtEntry;

/* End of Header Block */

#if !defined (VOSD_NO_FIN)
#undef	BIN
#undef	BOUT
#define	BIN		FIN_LOCAL_FIELD(_op_last_line_passed) = __LINE__ - _op_block_origin;
#define	BOUT	BIN
#define	BINIT	FIN_LOCAL_FIELD(_op_last_line_passed) = 0; _op_block_origin = __LINE__;
#else
#define	BINIT
#endif /* #if !defined (VOSD_NO_FIN) */



/* State variable definitions */
typedef struct
	{
	/* Internal state tracking for FSM */
	FSM_SYS_STATE
	/* State Variables */
	Objid	                  		process_id                                      ;
	Objid	                  		node_id                                         ;
	char	                   		node_type[10]                                   ;
	int	                    		node_address                                    ;
	List *	                 		routing_table_list                              ;
	int	                    		mySeqNo                                         ;
	int	                    		myBroadcastID                                   ;
	int	                    		RREQ_RETRIES                                    ;
	RequestSentAttributes	  		RequestSent[N]                                  ;	/* RequestSent repository. RequestSent[i] contains infomation           */
	                        		                                                	/* concerning the RREQ that has been generate for destination "i'. This */
	                        		                                                	/* includes dest seq nb, number of retries, expiration time, etc.       */
	RequestSeenAttributes	  		RequestSeen[N][N]                               ;
	int	                    		TTL_START                                       ;
	double	                 		ACTIVE_ROUTE_TIMEOUT                            ;
	double	                 		NODE_TRAVERSAL_TIME                             ;
	int	                    		TTL_INCREMENT                                   ;
	int	                    		TTL_TRESHOLD                                    ;
	int	                    		NET_DIAMETER                                    ;
	double	                 		Wait_ACK                                        ;	/* Maximum time that node should spend waiting for a data packet ackowledgement. */
	double	                 		DELETE_PERIOD                                   ;
	double	                 		MY_ROUTE_TIMEOUT                                ;
	List *	                 		ackEvt_list                                     ;
	} AODV_state;

#define process_id              		op_sv_ptr->process_id
#define node_id                 		op_sv_ptr->node_id
#define node_type               		op_sv_ptr->node_type
#define node_address            		op_sv_ptr->node_address
#define routing_table_list      		op_sv_ptr->routing_table_list
#define mySeqNo                 		op_sv_ptr->mySeqNo
#define myBroadcastID           		op_sv_ptr->myBroadcastID
#define RREQ_RETRIES            		op_sv_ptr->RREQ_RETRIES
#define RequestSent             		op_sv_ptr->RequestSent
#define RequestSeen             		op_sv_ptr->RequestSeen
#define TTL_START               		op_sv_ptr->TTL_START
#define ACTIVE_ROUTE_TIMEOUT    		op_sv_ptr->ACTIVE_ROUTE_TIMEOUT
#define NODE_TRAVERSAL_TIME     		op_sv_ptr->NODE_TRAVERSAL_TIME
#define TTL_INCREMENT           		op_sv_ptr->TTL_INCREMENT
#define TTL_TRESHOLD            		op_sv_ptr->TTL_TRESHOLD
#define NET_DIAMETER            		op_sv_ptr->NET_DIAMETER
#define Wait_ACK                		op_sv_ptr->Wait_ACK
#define DELETE_PERIOD           		op_sv_ptr->DELETE_PERIOD
#define MY_ROUTE_TIMEOUT        		op_sv_ptr->MY_ROUTE_TIMEOUT
#define ackEvt_list             		op_sv_ptr->ackEvt_list

/* These macro definitions will define a local variable called	*/
/* "op_sv_ptr" in each function containing a FIN statement.	*/
/* This variable points to the state variable data structure,	*/
/* and can be used from a C debugger to display their values.	*/
#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE
#define FIN_PREAMBLE_DEC	AODV_state *op_sv_ptr;
#define FIN_PREAMBLE_CODE	\
		op_sv_ptr = ((AODV_state *)(OP_SIM_CONTEXT_PTR->_op_mod_state_ptr));


/* Function Block */

#if !defined (VOSD_NO_FIN)
enum { _op_block_origin = __LINE__ + 2};
#endif

/////////////////////////////////////////////////////////////////
/****************  aodv_pk_send_to_mac_layer () ****************/
/////////////////////////////////////////////////////////////////

void 
aodv_pk_send_to_mac_layer(Packet *pk, int nextHop)
{
/*
/* Send packet (pk) to the MAC layer interface. The routine also 
/* installs an ICI in order to indicate the next hop address to 
/* which the packet should be sent.
*/
FIN(aodv_pk_send_to_mac_layer(Packet *pk, int nextHop));

op_pk_nfd_set(pk,"Previous Hop",node_address);
op_pk_nfd_set(pk,"Next Hop",nextHop);
op_pk_nfd_set(pk,"TR_source",node_id);

op_pk_send(pk,SEND_STRM);

FOUT;
}
/***********************************************************/
/******************* MAX int    ****************************/
/***********************************************************/
int 
max_int(int a, int b)
{
	  int result=a;
	  FIN(max_int(int a, int b));
	/*
	/* Retun the max between two integers
	*/
	if(b>a)
		result=b;
	FRET(result);
}

double 
max_dble(double a, double b)
{
	double result = a;
	FIN(max_dble(double a, double b));
	/*
	/* Retun the max between two doubles
	*/

	if(b>a) result=b;
	FRET(result);
}

RoutingTableEntry *
aodv_routingTable_entry_get(int destination)
{
//var
	//int i,rt_size;
	RoutingTableEntry* tb_entry;
//in
	FIN(aodv_routingTable_entry_get(int destination));
//body
	tb_entry=OPC_NIL;
	//待续
//out
	FRET(OPC_NIL);
}

int 
aodv_entry_nextHop_get(int destination)
{
//var
	int next_hop;
	//RoutingTableEntry* tb_entry;
//in
	FIN(aodv_entry_nextHop_get(int destination));
//body
	//待续
	next_hop=0;
//out
	FRET(next_hop);
}

int 
aodv_entry_destSeqNb_get(int destination) 
{
int result;
FIN(aodv_entry_destSeqNb_get(int destination));
/*
/* Return the destination sequence number for
/* the given entry.
*/
result=0;
if(aodv_routingTable_entry_get(destination) != OPC_NIL)
{
	result = (*aodv_routingTable_entry_get(destination)).destSeqNo; 
}
FRET(result);
}

void 
aodv_requestSent_repository_update(int destination, Request_Status_Type status)
{
    /*
	/* When a request is sent, a corresponding entry is      
	/* updated in the RequestSent repository in order 
	/* to control route requests broadcast. For each request    
	/* that has been generated, node stores the destination
	/* sequence number, the number of retries, the status
	/* of the request, the expiration time (time before 
	/* renewal), and finally the future TTL to use in case 
	/* of renewal
	*/
//var
//in
	FIN(aodv_requestSent_repository_update(int destination, Request_Status_Type status));
//body
	// Store the dest seq nb of the request
	RequestSent[destination].sequence_number=aodv_entry_destSeqNb_get(destination);
	// Set the status of the request 
	RequestSent[destination].status = status;
	// schedule for interruption if no reply received within appropriate period
	RequestSent[destination].evt = op_intrpt_schedule_remote(op_sim_time()+2*RequestSent[destination].ttl*NODE_TRAVERSAL_TIME,NREPLY_CODE+destination,op_id_self());
	RequestSent[destination].expirationTime= op_sim_time()+ 2*RequestSent[destination].ttl* NODE_TRAVERSAL_TIME;
	
	// increment TTL for next eventual renewal 
	RequestSent[destination].ttl=RequestSent[destination].ttl+TTL_INCREMENT;
	if(RequestSent[destination].ttl > TTL_TRESHOLD)
	{
		RequestSent[destination].ttl= NET_DIAMETER;
		// Increment the number of retries
		RequestSent[destination].nbOfRetries++;
	}
//out
	FOUT;
}

void 
aodv_rreq_pk_generate(int destination, Request_Status_Type status) 
{
//var
Packet*       pk;
//in
FIN(aodv_rreq_pk_generate(int destination, Request_Status_Type status));
//body
	printf("- Function aodv_rreq_pk_generate(destination %d)\n", destination);
	if(RequestSent[destination].status == OFF)
	{
		pk=op_pk_create_fmt("RREQ");
		op_pk_nfd_set(pk,"TTL",RequestSent[destination].ttl);
		op_pk_nfd_set(pk,"Originator",node_address);
		op_pk_nfd_set(pk,"Destination",destination);
		op_pk_nfd_set(pk,"Destination SeqNo",aodv_entry_destSeqNb_get(destination));
		op_pk_nfd_set(pk,"Originator SeqNo",mySeqNo);
		op_pk_nfd_set(pk,"RREQ ID",myBroadcastID);
		myBroadcastID++;
		
		printf("      > RREQ packet has been generated.\n");
		aodv_pk_send_to_mac_layer(pk, BROADCAST);
		// update RequestSent repository
		aodv_requestSent_repository_update(destination, status);
	}else
	{
		printf("      > A request is pending: cannot renew it.\n");
	}
	printf("- Function aodv_rreq_pk_generate() done.\n");
//out
	FOUT;
}

void 
aodv_ack_timeout_schedule(Packet* pk, int destination)
{

AckEvt*   ackEvt;
AckEvtEntry* ackEvtEntry;
FIN(aodv_ack_timeout_schedule(Packet* pk, int destination));
/*
/* This routine is called when a data packet is sent to the 
/* MAC Layer. As a matter of fact, this procedure schedules 
/* an interruption in Wait_Ack seconds of the current time. 
/* If no ack is received within this period of time, the 
/* process transits to the Ack_Timeout State in order to re-
/* transmit the lost packet.
/* However, if an ack is received, the ack-timeout interruption
/* is cancelled (see Rcv_Ack State).
*/

printf("- Function aodv_ack_timeout_schedule(destination %d)\n", destination);
// create an AckEvt structure to store interruption attributes
ackEvt= (AckEvt*) op_prg_mem_alloc(sizeof(AckEvt));
ackEvtEntry = (AckEvtEntry*)op_prg_mem_alloc(sizeof(AckEvtEntry));
if (ackEvt!=NULL)
	{
	// Schedule interruption with the appropriate code (destination to which packet has been sent)
	ackEvt->evt=op_intrpt_schedule_remote(op_sim_time()+ Wait_ACK,destination,op_id_self());
	// store a copy of the sent packet
	ackEvt->copy_pk_ptr = op_pk_copy(pk);
	ackEvtEntry->ackEvt=ackEvt;
	ackEvtEntry->listnumber=destination;
	// Store the Ack evt in the AckEvt list
	op_prg_list_insert(ackEvt_list,ackEvtEntry,OPC_LISTPOS_TAIL);
	}
FOUT;
}

void
aodv_entryPtr_expirationInterrupt_cancel(RoutingTableEntry * entry)
{
	// vars 
	//Evhandle*  evhandlePtr;
	FIN(aodv_entryPtr_expirationInterrupt_cancel(RoutingTableEntry * entry));
	/*
	/* Cancel the current expiration timeout if any is pending.
	*/
	if(entry->status != UNDER_REPAIR)
		{
		// Cancel the previous interruption
		if(entry->lastExpirationTime > op_sim_time())
			{
				printf("      > Event still pending: process cancels it\n");
				op_ev_cancel(entry->expiration_evt);
				printf("      > Evhandle cancelled\n");
			}
		else
			{
				printf("      > Event already occured\n");
			}		
		}
	FOUT;
}

void 
aodv_entryPtr_expirationInterrupt_schedule(RoutingTableEntry* entry)
{
	
	/*
	/* Remove the current expiration event associated 
	/* to the entryexpirationTime and schedule a new 
	/* expiration event for the next expirationTime.
	*/
	FIN(aodv_entryPtr_expirationInterrupt_schedule(RoutingTableEntry* entry));
	printf("- Function aodv_entryPtr_expirationInterrupt_schedule(): entry= %d \n", entry->dest);

	// Cancel the previous interruption 
	aodv_entryPtr_expirationInterrupt_cancel(entry);
	// schedule new interruption
	entry->expiration_evt = op_intrpt_schedule_self(entry->expirationTime,entry->dest); 
	printf("- Function aodv_entryPtr_expirationInterrupt_schedule()... done !\n");
	FOUT;
}

void 
aodv_entryPtr_expirationTime_set(RoutingTableEntry *entryPtr, double expirationTime)
{
	/*
	/* Set the expiration time of a given pointer
	/* to entry.
	*/
	FIN(aodv_entryPtr_expirationTime_set(RoutingTableEntry *entryPtr, double expirationTime));
	
	printf("- Refresh timeout for entry %d (+ %f)\n", entryPtr->dest, (expirationTime-op_sim_time()));
	// copy the expirationTime value to the lastExpirationTime field
	entryPtr->lastExpirationTime =  entryPtr->expirationTime;
	// Copy the new expiration time the expirationTime field
	entryPtr->expirationTime = expirationTime;
	// Schedule new interruption for entry invalidation
	aodv_entryPtr_expirationInterrupt_schedule(entryPtr);
	FOUT;
}


void 
aodv_entry_expirationTime_set(int destination, double expirationTime)
{
	/*
	/* Set the expiration time of a given entry 
	*/
	FIN(aodv_entry_expirationTime_set(int destination, double expirationTime));
	
	if(aodv_routingTable_entry_get(destination) != OPC_NIL)
		aodv_entryPtr_expirationTime_set(aodv_routingTable_entry_get(destination),expirationTime);
	FOUT;	
}

void 
aodv_data_pk_route(Packet* pk) 
{
//var
	RoutingTableEntry* tb_entry;
	int dest,nextHop;
//in
	FIN(aodv_data_pk_route(Packet* pk));
//body
	op_pk_nfd_get(pk,"Destination",&dest);
	printf("- Function aodv_data_pk_route(destination %d)\n", dest);
		
	if(op_subq_empty(dest) == OPC_FALSE)//DATA QUEUE IS NOT EMPTY
	{
		op_subq_pk_insert (dest,pk,OPC_QPOS_TAIL);
	}else
	{
		//check routing table for next hop
		nextHop = aodv_entry_nextHop_get(dest);
		if((nextHop != NOT_VALID) && (nextHop != NON_EXISTENT))
		{
		//node has an active entry for dest
			printf("      > Node has active fresh enough entry: packet is sent [next hop is %d]\n", nextHop);
			
			printf("      > Schedule ack-timeout event\n");
			aodv_ack_timeout_schedule(pk,dest);
			// send packet to mac layer
			aodv_pk_send_to_mac_layer(pk,nextHop);
			// refresh entry timeout
			printf("      > Refresh expiration time for entry\n");
			aodv_entry_expirationTime_set(dest,op_sim_time()+ACTIVE_ROUTE_TIMEOUT);
		}else
		{
			printf("	>No entry available: Data packet is inserted into buffer.\n");
			op_subq_pk_insert (dest,pk,OPC_QPOS_TAIL);
			if(RequestSent[dest].status==OFF)
			{
				printf("	>No request pending: Generate RREQ packet\n");
				if(nextHop == NOT_VALID)//Entry invalid: Set TTL to last known Hop Count
				{
					tb_entry = aodv_routingTable_entry_get(dest);
					RequestSent[dest].ttl = max_int(tb_entry->lastHopCount, TTL_START);
				}
				//Generate RREQ
				aodv_rreq_pk_generate(dest, WAITING_FOR_REPLY);
			}else
			{
				// request is either under repair or waiting for a reply
				printf("	>A request has already been sent: node waiting for reply.\n");
			}
		}
	}
//out
	FOUT;
}


/* End of Function Block */

/* Undefine optional tracing in FIN/FOUT/FRET */
/* The FSM has its own tracing code and the other */
/* functions should not have any tracing.		  */
#undef FIN_TRACING
#define FIN_TRACING

#undef FOUTRET_TRACING
#define FOUTRET_TRACING

#if defined (__cplusplus)
extern "C" {
#endif
	void AODV (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Obtype _op_AODV_init (int * init_block_ptr);
	void _op_AODV_diag (OP_SIM_CONTEXT_ARG_OPT);
	void _op_AODV_terminate (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Address _op_AODV_alloc (VosT_Obtype, int);
	void _op_AODV_svar (void *, const char *, void **);


#if defined (__cplusplus)
} /* end of 'extern "C"' */
#endif




/* Process model interrupt handling procedure */


void
AODV (OP_SIM_CONTEXT_ARG_OPT)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	FIN_MT (AODV ());

		{
		/* Temporary Variables */
		Packet* pk_TV;
		
		int i,j;
		int table_size;
		RoutingTableEntry* routing_tb_entry;
		
		/*
		//File
		FILE *in;
		char temp_file_name[300];
		*/
		/* End of Temporary Variables */


		FSM_ENTER ("AODV")

		FSM_BLOCK_SWITCH
			{
			/*---------------------------------------------------------*/
			/** state (init) enter executives **/
			FSM_STATE_ENTER_FORCED_NOLABEL (0, "init", "AODV [init enter execs]")
				FSM_PROFILE_SECTION_IN ("AODV [init enter execs]", state0_enter_exec)
				{
				process_id = op_id_self();
				node_id = op_topo_parent(process_id);
				
				op_ima_obj_attr_get(process_id,"ACTIVE_ROUTE_TIMEOUT",&ACTIVE_ROUTE_TIMEOUT);
				op_ima_obj_attr_get(process_id,"TTL",&TTL_START);
				
				op_ima_obj_attr_get(node_id, "Node Type",node_type);
				op_ima_obj_attr_get(node_id, "user id", &node_address);
				routing_table_list=op_prg_list_create();
				ackEvt_list = op_prg_list_create();
					
				NODE_TRAVERSAL_TIME = 1;
				TTL_INCREMENT       = 2;
				TTL_TRESHOLD        = 15;
				NET_DIAMETER          = 20;
				DELETE_PERIOD         = 5*ACTIVE_ROUTE_TIMEOUT;
				MY_ROUTE_TIMEOUT      = 2* ACTIVE_ROUTE_TIMEOUT;
				Wait_ACK              = DELETE_PERIOD;
				
				myBroadcastID         = 0;
				mySeqNo               = 0;
				
				for (i=0;i<N;i++)
				{
					RequestSent[i].sequence_number=0;
					RequestSent[i].status=OFF;
					RequestSent[i].nbOfRetries=0;
					RequestSent[i].ttl=TTL_START;
					//RequestSent[i].gratuitous =0;
					
					for(j=0;j<N;j++)
				    	{
						RequestSeen[i][j].expirationTime = -1;
						RequestSeen[i][j].broadcastID = -1;
						}
				}
				}
				FSM_PROFILE_SECTION_OUT (state0_enter_exec)

			/** state (init) exit executives **/
			FSM_STATE_EXIT_FORCED (0, "init", "AODV [init exit execs]")


			/** state (init) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "init", "idle", "tr_0", "AODV [init -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (idle) enter executives **/
			FSM_STATE_ENTER_UNFORCED (1, "idle", state1_enter_exec, "AODV [idle enter execs]")

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (3,"AODV")


			/** state (idle) exit executives **/
			FSM_STATE_EXIT_UNFORCED (1, "idle", "AODV [idle exit execs]")


			/** state (idle) transition processing **/
			FSM_PROFILE_SECTION_IN ("AODV [idle trans conditions]", state1_trans_conds)
			FSM_INIT_COND (END)
			FSM_TEST_COND (FROM_UPPER)
			FSM_TEST_COND (FROM_LOWER)
			FSM_TEST_COND (NO_REPLY)
			FSM_TEST_LOGIC ("idle")
			FSM_PROFILE_SECTION_OUT (state1_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 2, state2_enter_exec, ;, "END", "", "idle", "end", "tr_3", "AODV [idle -> end : END / ]")
				FSM_CASE_TRANSIT (1, 3, state3_enter_exec, ;, "FROM_UPPER", "", "idle", "DATA PK", "tr_4", "AODV [idle -> DATA PK : FROM_UPPER / ]")
				FSM_CASE_TRANSIT (2, 4, state4_enter_exec, ;, "FROM_LOWER", "", "idle", "LOWER PK", "tr_8", "AODV [idle -> LOWER PK : FROM_LOWER / ]")
				FSM_CASE_TRANSIT (3, 5, state5_enter_exec, ;, "NO_REPLY", "", "idle", "RENEW REQ", "tr_10", "AODV [idle -> RENEW REQ : NO_REPLY / ]")
				}
				/*---------------------------------------------------------*/



			/** state (end) enter executives **/
			FSM_STATE_ENTER_UNFORCED (2, "end", state2_enter_exec, "AODV [end enter execs]")
				FSM_PROFILE_SECTION_IN ("AODV [end enter execs]", state2_enter_exec)
				{
				
				//释放各种 list
				table_size=op_prg_list_size(routing_table_list);
				for(i=0;i<table_size;i++)
				{
					routing_tb_entry=op_prg_list_access(routing_table_list,i);
					op_prg_list_free (routing_tb_entry->listOfPrecursors);
					op_prg_mem_free (routing_tb_entry->listOfPrecursors);
				}
				
				op_prg_list_free (routing_table_list);
				op_prg_mem_free (routing_table_list);
				}
				FSM_PROFILE_SECTION_OUT (state2_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (5,"AODV")


			/** state (end) exit executives **/
			FSM_STATE_EXIT_UNFORCED (2, "end", "AODV [end exit execs]")


			/** state (end) transition processing **/
			FSM_TRANSIT_MISSING ("end")
				/*---------------------------------------------------------*/



			/** state (DATA PK) enter executives **/
			FSM_STATE_ENTER_FORCED (3, "DATA PK", state3_enter_exec, "AODV [DATA PK enter execs]")
				FSM_PROFILE_SECTION_IN ("AODV [DATA PK enter execs]", state3_enter_exec)
				{
				pk_TV=op_pk_get(SRC_STRM);
				aodv_data_pk_route(pk_TV);
				}
				FSM_PROFILE_SECTION_OUT (state3_enter_exec)

			/** state (DATA PK) exit executives **/
			FSM_STATE_EXIT_FORCED (3, "DATA PK", "AODV [DATA PK exit execs]")


			/** state (DATA PK) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "DATA PK", "idle", "tr_5", "AODV [DATA PK -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (LOWER PK) enter executives **/
			FSM_STATE_ENTER_FORCED (4, "LOWER PK", state4_enter_exec, "AODV [LOWER PK enter execs]")

			/** state (LOWER PK) exit executives **/
			FSM_STATE_EXIT_FORCED (4, "LOWER PK", "AODV [LOWER PK exit execs]")


			/** state (LOWER PK) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "LOWER PK", "idle", "tr_9", "AODV [LOWER PK -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (RENEW REQ) enter executives **/
			FSM_STATE_ENTER_FORCED (5, "RENEW REQ", state5_enter_exec, "AODV [RENEW REQ enter execs]")

			/** state (RENEW REQ) exit executives **/
			FSM_STATE_EXIT_FORCED (5, "RENEW REQ", "AODV [RENEW REQ exit execs]")


			/** state (RENEW REQ) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "RENEW REQ", "idle", "tr_11", "AODV [RENEW REQ -> idle : default / ]")
				/*---------------------------------------------------------*/



			}


		FSM_EXIT (0,"AODV")
		}
	}




void
_op_AODV_diag (OP_SIM_CONTEXT_ARG_OPT)
	{
	/* No Diagnostic Block */
	}




void
_op_AODV_terminate (OP_SIM_CONTEXT_ARG_OPT)
	{

	FIN_MT (_op_AODV_terminate ())


	/* No Termination Block */

	Vos_Poolmem_Dealloc (op_sv_ptr);

	FOUT
	}


/* Undefine shortcuts to state variables to avoid */
/* syntax error in direct access to fields of */
/* local variable prs_ptr in _op_AODV_svar function. */
#undef process_id
#undef node_id
#undef node_type
#undef node_address
#undef routing_table_list
#undef mySeqNo
#undef myBroadcastID
#undef RREQ_RETRIES
#undef RequestSent
#undef RequestSeen
#undef TTL_START
#undef ACTIVE_ROUTE_TIMEOUT
#undef NODE_TRAVERSAL_TIME
#undef TTL_INCREMENT
#undef TTL_TRESHOLD
#undef NET_DIAMETER
#undef Wait_ACK
#undef DELETE_PERIOD
#undef MY_ROUTE_TIMEOUT
#undef ackEvt_list

#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE

#define FIN_PREAMBLE_DEC
#define FIN_PREAMBLE_CODE

VosT_Obtype
_op_AODV_init (int * init_block_ptr)
	{
	VosT_Obtype obtype = OPC_NIL;
	FIN_MT (_op_AODV_init (init_block_ptr))

	obtype = Vos_Define_Object_Prstate ("proc state vars (AODV)",
		sizeof (AODV_state));
	*init_block_ptr = 0;

	FRET (obtype)
	}

VosT_Address
_op_AODV_alloc (VosT_Obtype obtype, int init_block)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	AODV_state * ptr;
	FIN_MT (_op_AODV_alloc (obtype))

	ptr = (AODV_state *)Vos_Alloc_Object (obtype);
	if (ptr != OPC_NIL)
		{
		ptr->_op_current_block = init_block;
#if defined (OPD_ALLOW_ODB)
		ptr->_op_current_state = "AODV [init enter execs]";
#endif
		}
	FRET ((VosT_Address)ptr)
	}



void
_op_AODV_svar (void * gen_ptr, const char * var_name, void ** var_p_ptr)
	{
	AODV_state		*prs_ptr;

	FIN_MT (_op_AODV_svar (gen_ptr, var_name, var_p_ptr))

	if (var_name == OPC_NIL)
		{
		*var_p_ptr = (void *)OPC_NIL;
		FOUT
		}
	prs_ptr = (AODV_state *)gen_ptr;

	if (strcmp ("process_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->process_id);
		FOUT
		}
	if (strcmp ("node_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->node_id);
		FOUT
		}
	if (strcmp ("node_type" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->node_type);
		FOUT
		}
	if (strcmp ("node_address" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->node_address);
		FOUT
		}
	if (strcmp ("routing_table_list" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->routing_table_list);
		FOUT
		}
	if (strcmp ("mySeqNo" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->mySeqNo);
		FOUT
		}
	if (strcmp ("myBroadcastID" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->myBroadcastID);
		FOUT
		}
	if (strcmp ("RREQ_RETRIES" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->RREQ_RETRIES);
		FOUT
		}
	if (strcmp ("RequestSent" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->RequestSent);
		FOUT
		}
	if (strcmp ("RequestSeen" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->RequestSeen);
		FOUT
		}
	if (strcmp ("TTL_START" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->TTL_START);
		FOUT
		}
	if (strcmp ("ACTIVE_ROUTE_TIMEOUT" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ACTIVE_ROUTE_TIMEOUT);
		FOUT
		}
	if (strcmp ("NODE_TRAVERSAL_TIME" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->NODE_TRAVERSAL_TIME);
		FOUT
		}
	if (strcmp ("TTL_INCREMENT" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->TTL_INCREMENT);
		FOUT
		}
	if (strcmp ("TTL_TRESHOLD" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->TTL_TRESHOLD);
		FOUT
		}
	if (strcmp ("NET_DIAMETER" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->NET_DIAMETER);
		FOUT
		}
	if (strcmp ("Wait_ACK" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->Wait_ACK);
		FOUT
		}
	if (strcmp ("DELETE_PERIOD" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->DELETE_PERIOD);
		FOUT
		}
	if (strcmp ("MY_ROUTE_TIMEOUT" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->MY_ROUTE_TIMEOUT);
		FOUT
		}
	if (strcmp ("ackEvt_list" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ackEvt_list);
		FOUT
		}
	*var_p_ptr = (void *)OPC_NIL;

	FOUT
	}

