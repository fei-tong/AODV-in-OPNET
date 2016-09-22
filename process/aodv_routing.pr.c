/* Process model C form file: aodv_routing.pr.c */
/* Portions of this file copyright 1986-2008 by OPNET Technologies, Inc. */



/* This variable carries the header into the object file */
const char aodv_routing_pr_c [] = "MIL_3_Tfile_Hdr_ 145A 30A modeler 7 4CD0F83B 4CD0F83B 1 china-0f9728557 Administrator 0 0 none none 0 0 none 0 0 0 0 0 0 0 0 1e80 8                                                                                                                                                                                                                                                                                                                                                                                            ";
#include <string.h>



/* OPNET system definitions */
#include <opnet.h>



/* Header Block */

#include <math.h>
#include "fifo.h"


/* Packet types */
#define DATA_PACKET_TYPE 5
#define REQUEST_PACKET_TYPE 7
#define REPLY_PACKET_TYPE 11
#define ERROR_PACKET_TYPE 13

/* Input and output streams */
#define SRC_STRM      0 // from upper layer
#define RCV_STRM      1 // from mac layer
#define SEND_STRM     0  // toward mac layer
#define DISCARD_STRM  1  // towards upper layer
#define BROADCAST    -1

/* Remote intrpt or self intrpt codes */
#define ACK_CODE      10000
#define NACK_CODE     20000
#define REP_CODE      30000//self intrp code in order to serve buffer
#define NREPLY_CODE   40000
#define HELLO_CODE    60000

/* Offset */
#define OPTION 10000

/* Maximum number of nodes in the net */
#define N 50

/* Transition macro */
#define END_SIM	               (op_intrpt_type() == OPC_INTRPT_ENDSIM)
#define NO_REPLY               (op_intrpt_type() == OPC_INTRPT_REMOTE && ((int)(op_intrpt_code()/OPTION))*OPTION == NREPLY_CODE)
#define UPPER_LAYER_ARVL       (op_intrpt_type() == OPC_INTRPT_STRM && op_intrpt_strm() == SRC_STRM)
#define LOWER_LAYER_ARVL       (op_intrpt_type() == OPC_INTRPT_STRM && op_intrpt_strm() == RCV_STRM)
#define ACK_ARVL               (op_intrpt_type() == OPC_INTRPT_REMOTE && op_intrpt_code() == ACK_CODE)
#define NACK_ARVL              (op_intrpt_type() == OPC_INTRPT_REMOTE && op_intrpt_code() == NACK_CODE)
#define TIMEOUT                (op_intrpt_type() == OPC_INTRPT_REMOTE &&  op_intrpt_code() < N)
#define Route_Expiration       (op_intrpt_type() == OPC_INTRPT_SELF && (op_intrpt_code() < N) )
#define Hello_Intvl_Expiration (op_intrpt_type() == OPC_INTRPT_SELF && (op_intrpt_code() == HELLO_CODE) )

/* infinity */
#define INFINITY -1
/* routing entry state */
#define NOT_VALID -1
#define NON_EXISTENT -2

/* request sent status */
typedef enum Request_Status_Type
	{
	OFF = 0, // Request waiting for Reply
	WAITING_FOR_REPLY = 1,     // Request was not sent
	WAITING_FOR_REPAIR = 2   // Request for repair
	} Request_Status_Type;

/* routing table entry status */
typedef enum Route_Status_Type
	{
	ACTIVE = 0,       // Entry is active
   	INVALID = 1,      // Entry is invalid
	UNDER_REPAIR = 2  // Entry is under repair. It must not be deleted.
	} Route_Status_Type;


/* Hello Module (used when HELLO MODE is activated) */
typedef struct
	{
	Evhandle evt;  
	Packet* hello_msg_template;
	} Hello_Module;

typedef struct
	{	
	int dest;
	int destSeqNb;
	int hopCount;
	int lastHopCount;
	int nextHop;
	sFifo* listOfPrecursors; 
	double expirationTime;
	double lastExpirationTime;
	Route_Status_Type status;
	Evhandle expiration_evt;
	} RoutingTableEntry; // a routing table entry 

typedef struct 
	{
	int broadcastID;
	double expirationTime;
	} RequestSeenAttributes; 

typedef struct
	{
	int dest;
	int destSeqNb;
	} UnreachableDestStruct;

typedef struct
	{
	int destCount;
	sFifo * listOfUnreachableDest;
	} ErrorContentStruct;

typedef struct
	{
	Evhandle evt;
	Packet* copy_pk_ptr;
	double schedule;
	} AckEvt;

typedef struct
	{
	int sequence_number;
	int nbOfRetries;
	double expirationTime;
	Request_Status_Type status;
	int ttl;
	int gratuitous;
	Evhandle evt;
	} RequestSentAttributes;



int aodv_buffer_size_get(int);

typedef struct
	{
	int own_input;
	int forward_input;
	int own_output;
	int forward_output;
	int data_pk_destroyed;
	int data_pk_buffer;
	int ack;
	} StatBlock;

StatBlock stats[50];
int input;
int output;
int data_pk_destroyed;
int data_pk_buffer;
int ack;
/* Function prototypes */

void aodv_pk_send_to_mac_layer(Packet *pk, int nextHop);
void aodv_hello_interval_extend();
void aodv_data_pk_route(Packet* data_pk_ptr) ;
void aodv_data_pk_queue(Packet* pk);
Boolean aodv_buffer_is_empty(int destination);
Packet* aodv_data_pk_dequeue(int destination);
void aodv_buffer_serve(int destination);
void aodv_ack_timeout_schedule(Packet* pk, int destination);
void aodv_rreq_pk_generate(int destination, Request_Status_Type status) ;
void aodv_requestSent_repository_update(int destination, Request_Status_Type status);
void aodv_rreq_pk_receive(Packet* rreq_pk_ptr);
void aodv_rrep_pk_generate_from_destination(Packet* rreq_pk_ptr);
void aodv_gratuitous_rrep_pk_generate(Packet* rreq_pk_ptr);
void aodv_rrep_pk_generate_from_relay(Packet* rreq_pk_ptr);
void aodv_rreq_pk_forward(Packet* rreq_pk_ptr);
void aodv_entry_update_or_create_from_rreq(Packet* rreq_pk_ptr);
void aodv_rreq_pk_regenerate(int destination);
void aodv_reverseListOfPrecursors_update(int precursor, int destination);
void aodv_data_pk_receive(Packet* data_pk_ptr);
RoutingTableEntry* aodv_entry_create_new();
void aodv_rerr_pk_generate(int unreachableNode, int n_flag);
void aodv_link_repair_attempt(int dest, int ttl_src);
void aodv_listOfPrecursors_node_remove(int precursor);
void aodv_listOfPrecursors_node_put(RoutingTableEntry* forwardEntryPtr,int previousHop);
void aodv_hello_msg_receive(Packet* rrep_pk_ptr);
void  aodv_rrep_pk_receive(Packet* rrep_pk_ptr);
void aodv_rrep_pk_forward(Packet *rrep_pk_ptr);
Boolean aodv_entry_update_or_create_from_rrep(Packet* rrep_pk_ptr);
Boolean aodv_entry_repair_from_rrep(Packet* rrep_pk_ptr);
void aodv_requestSent_repository_reset(int destination);
Boolean aodv_entry_update_from_rrep(Packet* rrep_pk_ptr);
Boolean aodv_entry_create_from_rrep(Packet* rrep_pk_ptr);
void aodv_rerr_pk_receive(Packet* rerr_pk_ptr);
void aodv_listOfUnreachableDest_insert(ErrorContentStruct
*newErrorStructPtr, int unreachableDest, int unreachableDestSeqNb);
int aodv_listOfUnreachableDest_dest_getFirst(ErrorContentStruct* errorStructPtr);
int aodv_listOfUnreachableDest_destSeqNb_getFirst(ErrorContentStruct *errorStructPtr);
Boolean aodv_pk_is_in_tr(Packet * pk_ptr);
void aodv_entryPtr_print(RoutingTableEntry* entryPtr);
void aodv_entry_print(int destination);
void aodv_pk_print(Packet* pk_ptr);
void aodv_unreachableDestList_print(sFifo fifo);
void aodv_routingTable_print();
int max_int(int a, int b); 
double max_dble(double a, double b);
void aodv_ack_print(int nextHop, int destination);
Boolean aodv_rreq_pk_is_fresh_enough(Packet* rreq_pk_ptr);
Boolean aodv_fresh_enough_entry_is_available(int destination, int destSeqNb);
RoutingTableEntry *aodv_routingTable_entry_get(int destination);
void aodv_routingTable_entry_delete(int dest);
void aodv_routingTable_entryPtr_put(RoutingTableEntry *newEntryPtr);
int aodv_entry_nextHop_get(int destination);
double aodv_entry_expirationTime_get(int destination);
void aodv_entry_expirationTime_set(int destination, double expirationTime);
void aodv_entryPtr_expirationTime_set(RoutingTableEntry *entryPtr, double expirationTime);
void aodv_entryPtr_hopCount_set(RoutingTableEntry *entryPtr, int newHopCount);
int aodv_entryPtr_hopCount_get(RoutingTableEntry *entryPtr);
int aodv_entry_hopCount_get(int dest);
void aodv_entryPtr_expirationInterrupt_schedule(RoutingTableEntry* entry);
void aodv_entry_invalidate(int dest, int destSeqNb, int n_flag);
Boolean aodv_entry_listOfPrecursors_is_empty(int dest);
void aodv_entry_listOfPrecursors_flush(int dest);
int aodv_entry_destSeqNb_get(int destination) ;
void aodv_entryPtr_destSeqNb_set(RoutingTableEntry *entryPtr,int destSeqNb);
void aodv_entryPtr_destination_set(RoutingTableEntry *entryPtr, int destination);
void aodv_entryPtr_nextHop_set(RoutingTableEntry *entryPtr, int nextHop);
void aodv_entryPtr_status_set(RoutingTableEntry *entryPtr, int status);
int aodv_entry_status_get(int dest);
void aodv_entryPtr_expirationInterrupt_cancel(RoutingTableEntry * entry);

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
	Objid	                  		node_id                                         ;	/* Current node ID */
	int	                    		node_addr                                       ;	/* Current node address */
	int	                    		net_id                                          ;	/* Current sub-network ID */
	double	                 		MY_ROUTE_TIMEOUT                                ;	/* AODV Constant (see AODV Specs) */
	double	                 		ACTIVE_ROUTE_TIMEOUT                            ;	/* AODV Constant (see AODV Specs) */
	int	                    		ALLOWED_HELLO_LOSS                              ;	/* AODV Constant (see AODV Specs) */
	double	                 		BROADCAST_RECORD_TIME                           ;	/* AODV Constant (see AODV Specs) */
	double	                 		HELLO_INTERVAL                                  ;	/* AODV Constant (see AODV Specs) */
	int	                    		LOCAL_ADD_TTL                                   ;	/* AODV Constant (see AODV Specs) */
	double	                 		MAX_REPAIR_TTL                                  ;	/* AODV Constant (see AODV Specs) */
	int	                    		MIN_REPAIR_TTL                                  ;	/* AODV Constant (see AODV Specs) */
	int	                    		NET_DIAMETER                                    ;	/* AODV Constant (see AODV Specs) */
	double	                 		NEXT_HOP_WAIT                                   ;	/* AODV Constant (see AODV Specs) */
	double	                 		NODE_TRAVERSAL_TIME                             ;	/* AODV Constant (see AODV Specs) */
	double	                 		NET_TRAVERSAL_TIME                              ;	/* AODV Constant (see AODV Specs) */
	double	                 		REV_ROUTE_LIFE                                  ;	/* AODV Constant (see AODV Specs) */
	int	                    		RREQ_RETRIES                                    ;	/* AODV Constant (see AODV Specs) */
	int	                    		TTL_INCREMENT                                   ;	/* AODV Constant (see AODV Specs) */
	int	                    		TTL_TRESHOLD                                    ;	/* AODV Constant (see AODV Specs) */
	int	                    		TTL_START                                       ;	/* AODV Constant (see AODV Specs) */
	double	                 		DELETE_PERIOD                                   ;	/* AODV Constant (see AODV Specs) */
	double	                 		TR                                              ;	/* Transmission Range */
	int	                    		DEBUG                                           ;	/* DEBUG flag to determine the depth of messages printing */
	int	                    		myBroadcastID                                   ;	/* Source broadcast ID */
	int	                    		mySeqNb                                         ;	/* Source sequence number */
	RequestSentAttributes	  		RequestSent[N]                                  ;	/* RequestSent repository. RequestSent[i] contains infomation           */
	                        		                                                	/* concerning the RREQ that has been generate for destination "i'. This */
	                        		                                                	/* includes dest seq nb, number of retries, expiration time, etc.       */
	RequestSeenAttributes	  		RequestSeen[N][N]                               ;	/* RequestSeen repository. RequestSeen[source][destination] contains     */
	                        		                                                	/* information about the RREQ that has been generated from node          */
	                        		                                                	/* "source" to node "destination". this helps the node to decide whether */
	                        		                                                	/* to answer a RREQ or not.                                              */
	                        		                                                	/*                                                                       */
	double	                 		Wait_ACK                                        ;	/* Maximum time that node should spend waiting for a data packet ackowledgement. */
	sFifo*	                 		routingTable                                    ;	/* Each node maintains its own routing table. A routing table is a list */
	                        		                                                	/* of routing table entries.                                            */
	sFifo*	                 		reverseListOfPrecursors                         ;	/* Each entry of this list contains the successor list of a given      */
	                        		                                                	/* node, given that if A is in the precursor list  of B, then B should */
	                        		                                                	/* be in the successor list of A                                       */
	sFifo	                  		ackEvtFifo                                      ;	/* List of Evhandles associated to ack-timeout expirations (list of */
	                        		                                                	/* destinations waiting for which the current node is expecting an  */
	                        		                                                	/* ack).                                                            */
	int	                    		HELLO_MODE                                      ;	/* Hello Mode flag. Indicates whether Hello Mode is enabled or not. */
	Hello_Module	           		hello_module                                    ;	/* Contains a template of the hello message to broadcast and a */
	                        		                                                	/* record of the last broadcast time.                          */
	Distribution *	         		hello_dist                                      ;	/* Distribution of the first hello interval. This avoids      */
	                        		                                                	/* synchronisation between nodes at the fisrt hello interval. */
	} aodv_routing_state;

#define node_id                 		op_sv_ptr->node_id
#define node_addr               		op_sv_ptr->node_addr
#define net_id                  		op_sv_ptr->net_id
#define MY_ROUTE_TIMEOUT        		op_sv_ptr->MY_ROUTE_TIMEOUT
#define ACTIVE_ROUTE_TIMEOUT    		op_sv_ptr->ACTIVE_ROUTE_TIMEOUT
#define ALLOWED_HELLO_LOSS      		op_sv_ptr->ALLOWED_HELLO_LOSS
#define BROADCAST_RECORD_TIME   		op_sv_ptr->BROADCAST_RECORD_TIME
#define HELLO_INTERVAL          		op_sv_ptr->HELLO_INTERVAL
#define LOCAL_ADD_TTL           		op_sv_ptr->LOCAL_ADD_TTL
#define MAX_REPAIR_TTL          		op_sv_ptr->MAX_REPAIR_TTL
#define MIN_REPAIR_TTL          		op_sv_ptr->MIN_REPAIR_TTL
#define NET_DIAMETER            		op_sv_ptr->NET_DIAMETER
#define NEXT_HOP_WAIT           		op_sv_ptr->NEXT_HOP_WAIT
#define NODE_TRAVERSAL_TIME     		op_sv_ptr->NODE_TRAVERSAL_TIME
#define NET_TRAVERSAL_TIME      		op_sv_ptr->NET_TRAVERSAL_TIME
#define REV_ROUTE_LIFE          		op_sv_ptr->REV_ROUTE_LIFE
#define RREQ_RETRIES            		op_sv_ptr->RREQ_RETRIES
#define TTL_INCREMENT           		op_sv_ptr->TTL_INCREMENT
#define TTL_TRESHOLD            		op_sv_ptr->TTL_TRESHOLD
#define TTL_START               		op_sv_ptr->TTL_START
#define DELETE_PERIOD           		op_sv_ptr->DELETE_PERIOD
#define TR                      		op_sv_ptr->TR
#define DEBUG                   		op_sv_ptr->DEBUG
#define myBroadcastID           		op_sv_ptr->myBroadcastID
#define mySeqNb                 		op_sv_ptr->mySeqNb
#define RequestSent             		op_sv_ptr->RequestSent
#define RequestSeen             		op_sv_ptr->RequestSeen
#define Wait_ACK                		op_sv_ptr->Wait_ACK
#define routingTable            		op_sv_ptr->routingTable
#define reverseListOfPrecursors 		op_sv_ptr->reverseListOfPrecursors
#define ackEvtFifo              		op_sv_ptr->ackEvtFifo
#define HELLO_MODE              		op_sv_ptr->HELLO_MODE
#define hello_module            		op_sv_ptr->hello_module
#define hello_dist              		op_sv_ptr->hello_dist

/* These macro definitions will define a local variable called	*/
/* "op_sv_ptr" in each function containing a FIN statement.	*/
/* This variable points to the state variable data structure,	*/
/* and can be used from a C debugger to display their values.	*/
#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE
#define FIN_PREAMBLE_DEC	aodv_routing_state *op_sv_ptr;
#define FIN_PREAMBLE_CODE	\
		op_sv_ptr = ((aodv_routing_state *)(OP_SIM_CONTEXT_PTR->_op_mod_state_ptr));


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

Ici*   ici_ptr;

/*
/* Send packet (pk) to the MAC layer interface. The routine also 
/* installs an ICI in order to indicate the next hop address to 
/* which the packet should be sent.
*/
FIN(aodv_pk_send_to_mac_layer(Packet *pk, int nextHop));
// set IP header 
// PreviousHop field = source IP address
op_pk_nfd_set(pk,"PreviousHop",node_addr);
// NextHop field = destination IP address
op_pk_nfd_set(pk,"NextHop",nextHop);
// set TR_source field for Transmission Range purpouse
op_pk_nfd_set(pk,"TR_source",node_id);
// print packet
if(DEBUG > 1) aodv_pk_print(pk);
// create and install an ici to communicate the packet destination 
// to the MAC layer interface.
ici_ptr = op_ici_create("aodv_notice_to_send");
op_ici_attr_set(ici_ptr,"Packet_Destination",nextHop);
op_ici_install(ici_ptr);
// send packet to mac layer
op_pk_send(pk,SEND_STRM);

if (DEBUG > 1) printf("      > Packet succesfully sent to MAC layer\n ");
FOUT;
}  

/////////////////////////////////////////////////////////////////
/*****************   aodv_data_pk_route()   *********************/
/////////////////////////////////////////////////////////////////

void 
aodv_data_pk_route(Packet* data_pk_ptr) 
{

RoutingTableEntry *   entryPtr;
int                   dest, g_flag, nextHop;

/*
/* Receive the upcoming data packet stream from the 
/* upper layer and route it to its destination.
/* Two scenarios are possible:
/* 1. Another packet destined for the same node
/* is waiting in the sub-queue. The procedure then
/* only enqueues the data packet in the sub-queue
/* and waits for the next call.
/* 2. Sub-queue is empty. The procedure checks its
/* routing table in order to find out the next hop
/* toward the destination. 
/* 2.a. If no entry was found, the procedure 
/* enqueues the packet in the corresponding sub-
/* queue. Then, the procedure checks whether a RREQ 
/* has been generated for this destination or not. 
/* If not, the procedure calls the appropriate function
/* to generate a RREQ. Else, it waits for the next call. 
/* 2.b. If an entry exists, the procedure send the
/* packet to the MAC layer with the appropriate next 
/* hop destination address.
*/
FIN(aodv_data_pk_route(Packet* data_pk_ptr));

// read packet
op_pk_nfd_get(data_pk_ptr,"DEST",&dest);
op_pk_nfd_get(data_pk_ptr,"G",&g_flag);


if(DEBUG > 1) printf("    - Function aodv_data_pk_route(destination %d)\n", dest);

if(aodv_buffer_is_empty(dest) == OPC_FALSE)
	{
	// enqueue data packet into appropriate buffer
	if(DEBUG > 1) printf("      > Buffer not empty: enqueue packet into buffer\n");
	aodv_data_pk_queue(data_pk_ptr);
	}
else
	{
	// check routing table for entry
	if(DEBUG > 1) printf("      > Check routing table for entry\n");
	entryPtr = aodv_routingTable_entry_get(dest);
	// print entry if any
	if(DEBUG > 1) aodv_entry_print(dest);
	// get next hop along the path if any	
	nextHop = aodv_entry_nextHop_get(dest);
	if ((nextHop != NOT_VALID) && (nextHop != NON_EXISTENT))
		{
		//node has an active entry for dest
		if(DEBUG > 1) printf("      > Node has active fresh enough entry: packet is sent [next hop is %d]\n", nextHop);
		// schedule an event if no ack received within the waiting window 
		if(DEBUG > 3) printf("      > Schedule ack-timeout event\n");
		aodv_ack_timeout_schedule(data_pk_ptr,dest);
		// send packet to mac layer
		aodv_pk_send_to_mac_layer(data_pk_ptr,nextHop);
		// refresh entry timeout
		if(DEBUG > 1 ) printf("      > Refresh expiration time for entry\n");
		aodv_entry_expirationTime_set(dest,op_sim_time()+ACTIVE_ROUTE_TIMEOUT); 
		}
	else
		{
		if(DEBUG > 1 ) printf("      > No entry available: Data packet is inserted into buffer.\n");
		// insert packet into buffer
		aodv_data_pk_queue(data_pk_ptr);
		// check whether node should generate a request or not 
		if(RequestSent[dest].status==OFF)
			{
			// No request pending for this destination
			// generate route request for desination 
			if(DEBUG > 1 ) printf("      > No request pending: Generate RREQ packet\n");
			RequestSent[dest].gratuitous = g_flag;
			if(nextHop == NOT_VALID)  // Entry invalid: Set TTL to last known Hop Count
				RequestSent[dest].ttl = max(entryPtr->lastHopCount,TTL_START);
			// Generate rreq
			aodv_rreq_pk_generate(dest, WAITING_FOR_REPLY);
			}
		else
			{
			// request is either under repair or waiting for a reply
			if(DEBUG > 1 ) printf("      > A request has already been sent: node waiting for reply\n");
			}
		}
	}
if(DEBUG > 1 ) printf("    - Function aodv_data_pk_route() done\n");
FOUT;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_data_pk_queue(Packet* pack)                    //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void 
aodv_data_pk_queue(Packet* pk)
{

int destination;

/*
/* Queue data packet in the corresponding sub-queue.
*/
FIN(aodv_data_pk_queue(Packet* pk));
// Read the DEST field (packet's final destination) in order to have index of subqueue
op_pk_nfd_get(pk,"DEST",&destination);
// Enqueue packet in the correct subqueue 
if (op_subq_pk_insert(destination,pk,OPC_QPOS_TAIL) != OPC_QINS_OK)
	{
	if(DEBUG > 3) printf("      > Fail to insert packet in subqueue \n");
	op_pk_destroy(pk);
	}
else
	{
	if(DEBUG > 1 ) printf("      > %d data packets waiting for destination %d\n", aodv_buffer_size_get(destination),destination);
	data_pk_buffer++;
	stats[node_addr].data_pk_buffer++;
	}
FOUT;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Boolean aodv_buffer_is_empty(int destination)            //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

Boolean 
aodv_buffer_is_empty(int destination)
	{

	/*
	/* Check whether the given sub-queue (the one whose
	/* index equals to destination) is empty or not.
	/* Return OPC_TRUE if buffer is empty. OPC_FALSE otherwise.
	*/
	FIN(aodv_buffer_is_empty(int destination));
	FRET(op_subq_empty(destination));
	
	}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// int aodv_buffer_size_get(int destination)               //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

int 
aodv_buffer_size_get(int destination)
	{

	/*
	/* Return the number of data packets waiting
	/* for this destination.
	*/
	FIN(aodv_buffer_size_get(int destination));
	FRET(op_subq_stat(destination, OPC_QSTAT_PKSIZE));
	
	}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Packet* aodv_data_pk_dequeue(int destination)             //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

Packet* 
aodv_data_pk_dequeue(int destination)
{

Packet* data_pk_ptr;

/*
/* Return the first packet (first in) from 
/* the indicated data buffer.
*/
FIN(aodv_data_pk_dequeue(int destination));
// If buffer not empty
if (aodv_buffer_is_empty(destination) == OPC_FALSE)
	{
	// get data packet from queue
	data_pk_ptr = op_subq_pk_remove(destination,OPC_QPOS_HEAD);
	// Decrement the number of data packets waiting into the buffer
	data_pk_buffer--;
	stats[node_addr].data_pk_buffer--;
	}
else 
	if(DEBUG > 3) printf("      > Buffer empty \n");

 FRET(data_pk_ptr);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_buffer_serve(int dest)                          //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void 
aodv_buffer_serve(int destination)
{

Packet*   data_pk_ptr;
int       nextHop = aodv_entry_nextHop_get(destination);
FIN(aodv_buffer_serve(int destination));
/*
/* This routine is called when a new entry is available for 
/* a given destination. When called, the procedure dequeues the first packet
/* waiting in the corresponding buffer, reads the next hop destination 
/* from the routing table, then sends the data packet to the MAC layer 
/* with the appropriate destination IP address.
*/

if(DEBUG > 1 ) printf("    - Function aodv_buffer_serve(destination %d)\n",destination);
if(! aodv_buffer_is_empty(destination))
{	
// Check routing table for entry
if(DEBUG > 1 ) printf("      > Check routing table for entry\n");
// print entry if any available
if(DEBUG >1) aodv_entry_print(destination);
// get packet from sub-queue
if(DEBUG > 1 ) printf("      > Get packet from buffer\n");

data_pk_ptr = aodv_data_pk_dequeue(destination);

if((nextHop != NOT_VALID) && (nextHop != NON_EXISTENT))
	{
	if(DEBUG > 1 ) printf("      > Node has a fresh active entry\n");
	// schedule an event if no ack received within the waiting window 
	if(DEBUG > 3 ) printf("      > Arming Ack Timer\n");
	aodv_ack_timeout_schedule(data_pk_ptr,destination);
	// send packet to mac layer
	aodv_pk_send_to_mac_layer(data_pk_ptr,nextHop);
	// refresh entry timeout
	if(DEBUG > 2 ) printf("      > Refresh expiration time for entry:\n");
	aodv_entry_expirationTime_set(destination,op_sim_time()+ACTIVE_ROUTE_TIMEOUT); 
	}
else
	{
	if(DEBUG > 1 ) printf("    > Error @ aodv_buffer_serve():: no valid entry was found !\n");
	// call route function
	aodv_data_pk_route(data_pk_ptr);
	}
}
else
	{
	if(DEBUG > 1 ) printf("    > Error @ aodv_buffer_serve():: buffer empty !\n");
	}
if(DEBUG > 1 ) printf("    - Function aodv_buffer_serve() done\n");
FOUT;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_ack_timeout_schedule(Packet* pk, int destination)  //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void 
aodv_ack_timeout_schedule(Packet* pk, int destination)
{

AckEvt*   ackEvt;
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

if(DEBUG > 3 ) printf("      - Function aodv_ack_timeout_schedule(destination %d)\n", destination);
// create an AckEvt structure to store interruption attributes
ackEvt= (AckEvt*) op_prg_mem_alloc(sizeof(AckEvt));
if (ackEvt!=NULL)
	{
	// Schedule interruption with the appropriate code (destination to which packet has been sent)
	ackEvt->evt=op_intrpt_schedule_remote(op_sim_time()+ Wait_ACK,destination,op_id_self());
	// store a copy of the sent packet
	ackEvt->copy_pk_ptr = op_pk_copy(pk);
	// Store the Ack evt in the AckEvt list
	putInFifoMultiplex(&ackEvtFifo,ackEvt,destination);
	}
FOUT;
}

/////////////////////////////////////////////////////////////////
/*********************** Generate RREQ *************************/
/////////////////////////////////////////////////////////////////

void 
aodv_rreq_pk_generate(int destination, Request_Status_Type status) 
{

Packet*       rreq_pk_ptr;
FIN(aodv_rreq_pk_generate(int destination, Request_Status_Type status));
/*
/* Initiate the discovery process by generating a RREQ for a given
/* destination.
/* The procedure not only generates a RREQ packet with the appropiate
/* fields. But it also updates node's "RequestSent" repository (see 
/* function aodv_requestSent_repository_update).
*/

if(DEBUG > 1) printf("    - Function aodv_rreq_pk_generate(destination %d)\n", destination);

// check if no request is already pending for that destination
if(RequestSent[destination].status == OFF)
	{
	// Create a request type packet 
	rreq_pk_ptr = op_pk_create_fmt("AODV_RREQ");
	// set RREQ fields
	op_pk_nfd_set(rreq_pk_ptr,"TTL",RequestSent[destination].ttl);
	op_pk_nfd_set(rreq_pk_ptr,"G",RequestSent[destination].gratuitous);
	op_pk_nfd_set(rreq_pk_ptr,"SRC",node_addr);
	op_pk_nfd_set(rreq_pk_ptr,"DEST",destination);
	op_pk_nfd_set(rreq_pk_ptr,"DestSeqNb",aodv_entry_destSeqNb_get(destination));
	op_pk_nfd_set(rreq_pk_ptr,"SrcSeqNb",mySeqNb);
	op_pk_nfd_set(rreq_pk_ptr,"BroadcastID",myBroadcastID);
	//increment node's boadcastID
	myBroadcastID++;
	// send to mac layer
	if(DEBUG > 1 ) printf("      > RREQ packet has been generated\n");
	aodv_pk_send_to_mac_layer(rreq_pk_ptr, BROADCAST);
	// update RequestSent repository
	aodv_requestSent_repository_update(destination, status);
	}
else 
	{
	if(DEBUG > 1 ) printf("      > A request is pending: cannot renew it\n");
	}
if(DEBUG > 1 ) printf("    - Function aodv_rreq_pk_generate() done\n");
FOUT;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_requestSent_repository_update(int dest  )  //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
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
	FIN(aodv_requestSent_repository_update(int destination, Request_Status_Type status));
	
	// Store the dest seq nb of the request
	RequestSent[destination].sequence_number=aodv_entry_destSeqNb_get(destination);
	// Set the status of the request 
	RequestSent[destination].status= status;
	
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
	FOUT;
	}


/////////////////////////////////////////////////////////////////
/*********************** HANDLE REQUEST ************************/
/////////////////////////////////////////////////////////////////

void 
aodv_rreq_pk_receive(Packet* rreq_pk_ptr)
{

int       dest, 
		  destSeqNb, 
		  source,G;

/*
/* Process the received RREQ. This procedure decides 
/* whether node should generate a RREP packet for the 
/* requested destination or simply forward it to the 
/* neighbouring nodes.
/* Note that RREQ is automatically discarded if seen 
/* less than BROADCAST_RECORD_TIME seconds ago
*/

FIN(aodv_rreq_pk_receive(Packet* rreq_pk_ptr));
if(DEBUG > 1) printf("    - Function aodv_rreq_pk_receive()\n");	

// reading packet 
op_pk_nfd_get (rreq_pk_ptr,"DEST",&dest); 
op_pk_nfd_get (rreq_pk_ptr,"SRC",&source);
op_pk_nfd_get (rreq_pk_ptr,"DestSeqNb",&destSeqNb);
op_pk_nfd_get (rreq_pk_ptr,"G",&G);

// check if RREQ has already been seen
if(aodv_rreq_pk_is_fresh_enough(rreq_pk_ptr) == OPC_TRUE)				
	{
	//request NEVER seen OR SEEN more than BROADCAST_RECORD_TIME ago
	if(DEBUG > 1 ) printf("      > RREQ is fresh enough: request handled \n");
	// update or create reverse entry 
	aodv_entry_update_or_create_from_rreq(rreq_pk_ptr);
	
	// handle request
	if(node_addr == dest) // the current node is the destination
		{
		if(DEBUG > 1 ) printf("      > RREQ has reached its destination: node replies\n");
		aodv_rrep_pk_generate_from_destination(rreq_pk_ptr);
		}
	else  // the node is an intermediate node
		{		
		if(DEBUG > 1 ) printf("      > Node is a relay\n");
		if(aodv_fresh_enough_entry_is_available(dest,destSeqNb)== OPC_TRUE)
			{
			if(DEBUG > 1) aodv_entry_print(dest);  
			// node has a fresh enough route
			if(DEBUG > 1 ) printf("      > Node has active fresh enough entry: node replies\n");
			// if gratuitous mode requested, generate gratuitous RREP for destination
			if(G)
				{
				if(DEBUG > 1 ) printf("      > Gratuitous reply requested: generate reply for destination\n");
				aodv_gratuitous_rrep_pk_generate(rreq_pk_ptr);
				}
			// generate rrep for source
			if(DEBUG > 1 ) printf("      > Generate reply for the source node\n");
 			aodv_rrep_pk_generate_from_relay(rreq_pk_ptr);
			}
		else // node doesn't have any active entry for the requestedroute
			{
			if(DEBUG > 1 ) printf("      > No active entry for requested route: RREQ should be forwarded\n");
			aodv_rreq_pk_forward(rreq_pk_ptr);
			}
		}
	// print reverse entry if any
	if(DEBUG > 1) printf("      > Print reverse entry if any available\n");
	if(DEBUG > 1) aodv_entry_print(source);
	// print forward entry if any
	if(DEBUG > 1) printf("      > Print forward entry if any available\n");
	if(DEBUG > 1) aodv_entry_print(dest);
	
	}
else //request seen less than BROADCAST_RECORD_TIME ago
	{
	// RREQ discarded 
	if(DEBUG > 1 ) printf("      > Request seen less than BRODCAST_RECORD_TIME ago: RREQ discarded\n");
	op_pk_destroy(rreq_pk_ptr);
	}
		
if(DEBUG > 1 ) printf("    - Function aodv_rreq_pk_receive() done\n");
FOUT;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_rrep_pk_generate_from_destination(Packet* rreq_pk_ptr)   //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void 
aodv_rrep_pk_generate_from_destination(Packet* rreq_pk_ptr)
	{
	
	Packet*     rrep_pk_ptr;	
	int         previousHop,        
		        source,
	            dest,
				destSeqNb;
	/*
	/* This function is called when a RREQ has reached its 
	/* destination and node wants to reply.
	/* Once the RREQ packet is generated, the procedure unicat
	/* it back to the node upstream (node from which RREQ was
	/* received) (PreviousHop field).
	*/
	FIN(aodv_rrep_pk_generate_from_destination(Packet* rreq_pk_ptr));
	// Read the received RREQ packet
	op_pk_nfd_get (rreq_pk_ptr,"SRC",&source);
	op_pk_nfd_get (rreq_pk_ptr,"DEST",&dest);
	op_pk_nfd_get (rreq_pk_ptr,"PreviousHop",&previousHop);
	op_pk_nfd_get (rreq_pk_ptr,"DestSeqNb",&destSeqNb);

	// Create reply packet
	rrep_pk_ptr = op_pk_create_fmt("AODV_RREP");
	// Assign source and dest to the reply packet
	op_pk_nfd_set(rrep_pk_ptr,"SRC",source);
	op_pk_nfd_set(rrep_pk_ptr,"DEST",dest);
	op_pk_nfd_set(rrep_pk_ptr,"HopCount",0);
	op_pk_nfd_set(rrep_pk_ptr,"Lifetime",MY_ROUTE_TIMEOUT);
	
	// Assign seq number  
	mySeqNb = max(destSeqNb, mySeqNb);
	op_pk_nfd_set(rrep_pk_ptr,"DestSeqNb",mySeqNb);
	if(DEBUG > 1 ) printf("      > RREP has been generated from destination: unicast it to source node [next hop is %d]\n", previousHop);
	// send reply packet 
	aodv_pk_send_to_mac_layer(rrep_pk_ptr,previousHop);
	// Refresh timeout
	aodv_entry_expirationTime_set(source,op_sim_time()+ACTIVE_ROUTE_TIMEOUT);
	// Destroy rreq 
	op_pk_destroy(rreq_pk_ptr);
	FOUT;
	}		   


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_gratuitous_rrep_pk_generate(Packet* rreq_pk_ptr)   //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void 
aodv_gratuitous_rrep_pk_generate(Packet* rreq_pk_ptr)
	{
	
	Packet*             grat_rrep_pk_ptr;
	RoutingTableEntry * forwardEntryPtr;
	int                 dest,
	                    source,
            		    srcSeqNb,
			            hopCount;
	double              lifetime;
	 
	/*
	/* Generate a gratuitous RREP and unicast it to
	/* the destination node (node for which RREQ was
	/* originally generated).
	*/
	FIN(aodv_gratuitous_rrep_pk_generate(Packet* rreq_pk_ptr));
	// Read RREQ packet
	op_pk_nfd_get (rreq_pk_ptr,"SRC",&source);
	op_pk_nfd_get (rreq_pk_ptr,"DEST",&dest); 
	op_pk_nfd_get (rreq_pk_ptr,"SrcSeqNb",&srcSeqNb);
	op_pk_nfd_get (rreq_pk_ptr,"HopCount",&hopCount);
	
	// read forward entry
	forwardEntryPtr = aodv_routingTable_entry_get(dest);
	// create a gratuitous rrep packet and unicast it to the next hop toward the Destination
	grat_rrep_pk_ptr= op_pk_create_fmt("AODV_RREP");
	op_pk_nfd_set(grat_rrep_pk_ptr,"SRC",dest);
	op_pk_nfd_set(grat_rrep_pk_ptr,"DEST",source);
	op_pk_nfd_set(grat_rrep_pk_ptr,"DestSeqNb",srcSeqNb);
	op_pk_nfd_set(grat_rrep_pk_ptr,"HopCount",hopCount+1);
	
	lifetime = forwardEntryPtr->expirationTime - op_sim_time();
	if(lifetime <= 0)
		lifetime = ACTIVE_ROUTE_TIMEOUT;
	op_pk_nfd_set(grat_rrep_pk_ptr,"Lifetime",lifetime);
	
	// send to MAC Layer
	aodv_pk_send_to_mac_layer(grat_rrep_pk_ptr,aodv_entry_nextHop_get(dest));
	// Refresh timeout
	aodv_entry_expirationTime_set(dest,op_sim_time()+ACTIVE_ROUTE_TIMEOUT);

	if (DEBUG > 1) printf("      > Gratuitous RREP has been generated: unicast it to destination [next hop is %d]\n", aodv_entry_nextHop_get(dest));
	FOUT;
	}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_rrep_pk_generate_from_relay(Packet*    )   //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void 
aodv_rrep_pk_generate_from_relay(Packet* rreq_pk_ptr)
	{

	Packet                *rrep_pk_ptr;
	double				  lifetime;
	RoutingTableEntry     *forwardEntryPtr;
	RoutingTableEntry     *reverseEntryPtr;
	int                   dest;
	int                   source;
	int					  previousHop;
	
	/*
	/* This routine is called when an intermediate node 
	/* receives a RREQ an has fresh enough route to 
	/* to generate a RREP for the requested destination. 
	*/
	FIN(aodv_rrep_pk_generate_from_relay(Packet* rreq_pk_ptr));
	// Read request packet
	op_pk_nfd_get (rreq_pk_ptr,"SRC",&source);
	op_pk_nfd_get (rreq_pk_ptr,"DEST",&dest); 
   op_pk_nfd_get (rreq_pk_ptr,"PreviousHop",&previousHop);

	// Read forward entry
	forwardEntryPtr=aodv_routingTable_entry_get(dest);
	// Create reply packet 
	rrep_pk_ptr = op_pk_create_fmt("AODV_RREP");
	/* Assign source and dest to the reply packet */
	op_pk_nfd_set(rrep_pk_ptr,"SRC",source);
	op_pk_nfd_set(rrep_pk_ptr,"DEST",dest);
	op_pk_nfd_set(rrep_pk_ptr,"HopCount",forwardEntryPtr->hopCount);

	lifetime = forwardEntryPtr->expirationTime - op_sim_time();
	if(lifetime <= 0)
		lifetime = ACTIVE_ROUTE_TIMEOUT;
		op_pk_nfd_set(rrep_pk_ptr,"Lifetime",lifetime);
	
	op_pk_nfd_set(rrep_pk_ptr,"DestSeqNb",forwardEntryPtr->destSeqNb);
	// add previousHop toward "source" to the precursor list of the forward entry
	aodv_listOfPrecursors_node_put(forwardEntryPtr,previousHop);
	// add nextHop toward "destination" to the precursor list of the reverse entry
	reverseEntryPtr=aodv_routingTable_entry_get(source);
	aodv_listOfPrecursors_node_put(reverseEntryPtr,forwardEntryPtr->nextHop);
	// send reply to mac layer
	if(DEBUG > 1 ) printf("      > RREP has been generated from relay: unicast it to source node [next hop is %d]\n",previousHop);
	aodv_pk_send_to_mac_layer(rrep_pk_ptr,previousHop);
	// Refresh timeout
	aodv_entry_expirationTime_set(source,op_sim_time()+ACTIVE_ROUTE_TIMEOUT);

	// Destroy rreq 
	op_pk_destroy(rreq_pk_ptr);
	FOUT;
	}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_rreq_pk_forward(Packet* rreq_pk_ptr)       //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
	
void 
aodv_rreq_pk_forward(Packet* rreq_pk_ptr)
	{
	
	int      ttl, 
	         hopCount;
	FIN(aodv_rreq_pk_forward(Packet* rreq_pk_ptr));
	/*
	/* Node does not have a fresh enough entry
	/* (or does not have an entry at all) to answer
	/* the received RREQ, so it decides to forward 
	/* it. Node increments the Hop Count field
	/* and decrements the TTL field. 
	/* Note that packet is sent to mac layer only if 
	/* TTL > 0.
	*/
	op_pk_nfd_get (rreq_pk_ptr,"TTL",&ttl);		
	op_pk_nfd_get (rreq_pk_ptr,"HopCount",&hopCount);

	if(ttl > 1)
		{
		if(DEBUG > 1 ) printf("      > Node forwards request\n",ttl);
		op_pk_nfd_set(rreq_pk_ptr,"TTL",ttl-1);
		op_pk_nfd_set(rreq_pk_ptr,"HopCount",hopCount+1);
		// send to mac
		aodv_pk_send_to_mac_layer(rreq_pk_ptr, BROADCAST);
		}
	else
		{
		if(DEBUG > 1 ) printf("      > TTL expired: rreq destroyed\n");
		op_pk_destroy(rreq_pk_ptr);
		}
	FOUT;
	}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_entry_update_or_create_from_rreq(Packet*     ) //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void 
aodv_entry_update_or_create_from_rreq(Packet* rreq_pk_ptr)
	{

	RoutingTableEntry *    reverseEntryPtr;
	int                    source,
	                       previousHop,
						   srcSeqNb,
						   hopCount;
	/*
	/* Update or create reverse entry following a RREQ reception.
	*/
	FIN(aodv_entry_update_or_create_from_rreq(Packet* rreq_pk_ptr));
	// Read RREQ packet
	op_pk_nfd_get (rreq_pk_ptr,"SRC",&source);
	op_pk_nfd_get (rreq_pk_ptr,"PreviousHop",&previousHop); 
	op_pk_nfd_get (rreq_pk_ptr,"SrcSeqNb",&srcSeqNb);
	op_pk_nfd_get (rreq_pk_ptr,"HopCount",&hopCount);
	
	// Check if any request is pending for this destination
	if(RequestSent[source].status != OFF)
		aodv_requestSent_repository_reset(source);
	
	// Node creates or updates a REVERSE route
	reverseEntryPtr=aodv_routingTable_entry_get(source);
	if(reverseEntryPtr != OPC_NIL) 
		{
		//entry already exists
		if((srcSeqNb > reverseEntryPtr->destSeqNb) ||((srcSeqNb == reverseEntryPtr->destSeqNb)
		&&((hopCount+1)< reverseEntryPtr->hopCount)) || (reverseEntryPtr->status != ACTIVE) )
			{
			// entry updated only if (greater seqNb) or 
			//(same seqNbs but smaller hopCount)
			if(DEBUG > 1 ) printf("      > Node updates its reverse route to source node\n");
			reverseEntryPtr->destSeqNb=srcSeqNb;
			reverseEntryPtr->nextHop= previousHop;
			if(reverseEntryPtr->hopCount != INFINITY)
				reverseEntryPtr->lastHopCount= reverseEntryPtr->hopCount;
			reverseEntryPtr->hopCount=hopCount+1;
			}
		else
			if(DEBUG > 1) printf("      > Node does not need to update its reverse entry\n");
		
		// Refresh entry timeout to max(current expiration time, currentTime+REV_ROUTE_LIFE)	
		if((op_sim_time()+REV_ROUTE_LIFE) > reverseEntryPtr->expirationTime)			
			{			
			aodv_entry_expirationTime_set(source,op_sim_time()+REV_ROUTE_LIFE);
			}
		else
			if(DEBUG > 1) printf("      > Reverse entry timeout does not need to be refreshened\n");

		// set entry status
		reverseEntryPtr->status = ACTIVE;
		}
	else  
		{
		//entry doesn't exist: node creates it
		if(DEBUG > 1 ) printf("      > Node creates a reverse entry to source node\n");
		reverseEntryPtr=aodv_entry_create_new();
		reverseEntryPtr->listOfPrecursors=newFifo();
		reverseEntryPtr->status = ACTIVE;
		reverseEntryPtr->dest=source;
		reverseEntryPtr->destSeqNb=srcSeqNb;
		reverseEntryPtr->nextHop= previousHop;
		reverseEntryPtr->hopCount=hopCount+1;
		reverseEntryPtr->lastHopCount = hopCount+1;
		// Add the entry to the RoutingTable
		aodv_routingTable_entryPtr_put(reverseEntryPtr);
		// Refresh entry timeout
		aodv_entry_expirationTime_set(source,op_sim_time()+REV_ROUTE_LIFE);
		}
	
	// if reverse entry updated or created, then
	// Serve buffer if any packet's waiting
	if(reverseEntryPtr->hopCount != INFINITY)
		aodv_buffer_serve(source);
	FOUT;
	}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_rreq_pk_regenerate(int destination)            //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void 
aodv_rreq_pk_regenerate(int destination)
{

/*
/* This function is called in the Renew_Request state when 
/* the previous RREQ didn't receive any RREP and the number
/* of retries is smaller than TTL_RETRIES_TRESHOLD
*/
FIN(aodv_rreq_pk_regenerate(int destination));
aodv_rreq_pk_generate(destination, WAITING_FOR_REPLY);
FOUT;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_reverseListOfPrecursors_update(int prec, int dest)       //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void 
aodv_reverseListOfPrecursors_update(int precursor, int destination)
	{

	int      *destinationPtr; 
	sFifo    *listOfDestination;
	FIN(aodv_reverseListOfPrecursors_update(int precursor, int destination));
	/*
	/* Each time a node "A" (precursor) is added to the list of precursor of 
	/* a given destination "B" (detination), this function is called to add
	/* "B" in the list of nodes to which "A" is a precursor. Thus, when 
	/* link to A is no longer available and node wants to expunge it from 
	/* its routing table, node has only to process entry that are listed
	/* within the reverseListOfPrecursol at the entry "A".
	*/
	
	// var init
	destinationPtr = (int*) op_prg_mem_alloc(sizeof(int));
	*destinationPtr = destination;
	// begin
	if(DEBUG > 3 ) printf("      - Function aodv_reverseListOfPrecursors_update(): precursor = %d || entry = %d\n", precursor,destination);
	// check if a reverse entry already exists for PRECURSOR
	if(DEBUG > 3 ) printf("      > check if a reverse entry already exists\n");
	listOfDestination= (sFifo*) readInFifoMultiplex(reverseListOfPrecursors, precursor);
	
	if(listOfDestination == OPC_NIL) // no entry available
		{
		if(DEBUG > 3 ) printf("      > No entry available: node has to create a new one\n");
		// have to create a new entry
		listOfDestination=newFifo();
		putInFifo(listOfDestination, destinationPtr);
		putInFifoMultiplex(reverseListOfPrecursors,listOfDestination, precursor);
		}
	else
		{
		// only add destination to the list of destination
		if(DEBUG > 3 ) printf("      > entry already exists: only add destination to the list of destination\n");
		putInFifo(listOfDestination, destinationPtr);
		}
	if(DEBUG > 3 ) printf("      - function aodv_reverseListOfPrecursors_update()... done !\n");
	FOUT;
   }

///////////////////////////////////////////////////////////
/***************** HANDLE_DATA ***************************/
///////////////////////////////////////////////////////////

void 
aodv_data_pk_receive(Packet* data_pk_ptr)
{
// var
int        previousHop, source, dest; 

/*
/* This function receives the upcoming data packet from the MAC Layer.
/* Two possibilities:
/* 1. Packet has reached its destination. Packet is then discarded toward
/*    the upper_layer.
/* 2. Packet has to be forwarded. Node checks its routing table and
/*    either forwards the packet if an entry is available or generates
/*    a RERR otherwise.
/* 
*/
FIN(aodv_data_pk_receive(Packet* data_pk_ptr));
// Read data packet 
op_pk_nfd_get (data_pk_ptr,"PreviousHop",&previousHop);
op_pk_nfd_get (data_pk_ptr,"SRC",&source);
op_pk_nfd_get (data_pk_ptr,"DEST",&dest); 

if(DEBUG > 1 ) printf("    - Function aodv_data_pk_receive(destination is %d )\n",dest);
// update stats
if(previousHop != source)
	{
	stats[previousHop].forward_output++;
	}

// check packet final destination
if (dest == node_addr)  
	{
	// pk has reached its destination
	if(DEBUG > 1 ) printf("      > Data packet has reached its destination\n");
	// send packet to upper layer 
	if(DEBUG > 1 ) printf("      > Data packet sent to app_manager layer\n");
	op_pk_send(data_pk_ptr,DISCARD_STRM); 
	// update stats
	output ++;
	stats[source].own_output++;
	}
else  
	{
	// pk has to be forwarded
	if(DEBUG > 1 ) printf("      > Node is a relay: packet has to be routed\n");
	// update forward input stat
	stats[node_addr].forward_input++;
	// check routing table for desired entry 
	if(DEBUG > 1) printf("      > Check routing table for entry\n");
	if(DEBUG > 1) aodv_entry_print(dest);
	if ((aodv_entry_nextHop_get(dest) != NOT_VALID) && (aodv_entry_nextHop_get(dest) != NON_EXISTENT))
		 {
		 // entry available: node forwards the packet 
		 if(DEBUG > 1 ) printf("      > Node has fresh enough active entry\n      > Data packet forwarded\n");
		 // schedule an event if no ack received within the waiting window 
		 if(DEBUG > 3) printf("      > Schedule Event if no ack received\n");
		 aodv_ack_timeout_schedule(data_pk_ptr,dest);
		 //send to mac
		 aodv_pk_send_to_mac_layer(data_pk_ptr,aodv_entry_nextHop_get(dest));
		 // re-active the route timeout (must delay the expiration event 
		 // which has been scheduled for the current entry)
		 aodv_entry_expirationTime_set(dest,op_sim_time()+ACTIVE_ROUTE_TIMEOUT); 
		 }
	 else if(aodv_entry_nextHop_get(dest) == NOT_VALID)
		 {
		 printf("      > Error @ node %d: No forward route available: no active entry for dest %d\n",node_addr,dest);
		 // insert data packet into buffer and try to repair the broken link
		 if(DEBUG > 1 ) printf("      > Insert packet into buffer\n");
		 aodv_data_pk_queue(data_pk_ptr);
		 // if no repair pending
		 if(RequestSent[dest].status != WAITING_FOR_REPAIR)
			 {
			 // cancel expiration time
			 aodv_entryPtr_expirationInterrupt_cancel(aodv_routingTable_entry_get(dest));
			 // attempt to repair
			 if(DEBUG > 1 ) printf("      > Attempt to repair link\n");
			 aodv_link_repair_attempt(dest, aodv_entry_hopCount_get(source));
			 }
		 }
	 else // entry does not exist
		 {
		 // generate a rreq
		 aodv_rreq_pk_generate(dest, WAITING_FOR_REPLY);
		 }
	 }
	if(DEBUG > 1 ) printf("    - Function aodv_data_pk_receive() done\n");
	FOUT;
}



/***********************************************************/
/***************** INIT ROUTING TABLE ENTRY ****************/
/***********************************************************/

RoutingTableEntry* 
aodv_entry_create_new()
{

RoutingTableEntry* entryPtr;
FIN(aodv_entry_create_new());
/*
/* Returns a new routing table entry with the default
/* values.
*/

entryPtr= (RoutingTableEntry*) op_prg_mem_alloc(sizeof(RoutingTableEntry));

entryPtr->status             = ACTIVE;
entryPtr->dest               = -1;
entryPtr->nextHop            = -1;
entryPtr->destSeqNb          =  0; 
entryPtr->hopCount           = -1;
entryPtr->lastHopCount       = -1;
entryPtr->expirationTime     =  0;
entryPtr->lastExpirationTime =  0;

FRET(entryPtr);
}

/////////////////////////////////////////////////////////////////
/*********************** GENERATE ERROR ************************/
/////////////////////////////////////////////////////////////////
void 
aodv_rerr_pk_generate(int unreachableNode, int n_flag)
{
// vars
RoutingTableEntry *  indexEntryError;
ErrorContentStruct*  newErrorStructPtr; 
sObject*             object;
Packet*              rerr_pk_ptr;
//int                  source,
//                     dest,
//					 destSeqNb,
int					 unreachableDest,
					 unreachableDestSeqNb,
					 i;
/*
/* Generate a RERR packet containing the list of all
/* destinations (and their associated Dest. Seq. Nb.)
/* that are now unreachable because of the loss of 
/* node given in the first parameter (unreachableNode). 
/* The N flag (n_flag) indicates whether entry should 
/* be deleted or not.
*/
FIN(aodv_rerr_pk_generate(int unreachableNode, int n_flag));
//init vars
object=  routingTable->firstObject;
newErrorStructPtr= (ErrorContentStruct *) 
		 				op_prg_mem_alloc(sizeof(ErrorContentStruct));
newErrorStructPtr->listOfUnreachableDest=newFifo();

// Begin
if(DEBUG > 1) printf("    - Function aodv_rerr_pk_generate(destination %d)\n", unreachableNode);
// Read corresponding entry
indexEntryError = aodv_routingTable_entry_get(unreachableNode);
// check the non_delete flag
if(n_flag == 0)
	{
	// If not set, invalidate entry and go through all listOfPrecursors and remove the unreachable 
	// destination from them: this is done via reverseListOfPrecursors
	// invalidate entry (dest seq nb was previously incremented while attempting to repair)
	// this function will also schedule deletion for the current entry
	if(DEBUG > 1) printf("    - Invalidate entry\n");
	aodv_entry_invalidate(indexEntryError->dest, indexEntryError->destSeqNb, n_flag);
	if(DEBUG > 1 ) printf("      > Remove unreachable node from all the precursor lists\n");
	aodv_listOfPrecursors_node_remove(indexEntryError->dest);
	}
// add the new unreachable destination to the RERR packet
if(DEBUG > 1 ) printf("      > Add destination to RERR packet\n");
// add destination to the list of new unreachable destinations 
aodv_listOfUnreachableDest_insert(newErrorStructPtr, indexEntryError->dest, indexEntryError->destSeqNb);

// If non_delete flag is not set, process the rest of routingTable entries:
// go through all entries, and process only those whos nextHop is the unreachable node
if(DEBUG > 1) printf("    - Look for other lost destinations\n");
if(n_flag == 0)
	{
	for(i=0;i<getFifoSize(routingTable);i++)
		{
		// read entry
		indexEntryError = (RoutingTableEntry*)(object->data);
		// Proceed only if the unreachable node is the next hop for this destination
		if(indexEntryError->nextHop == unreachableNode && indexEntryError->dest != unreachableNode) 
			{
			// read entry 
			unreachableDest = indexEntryError->dest;
			unreachableDestSeqNb = indexEntryError->destSeqNb;
			
			// invalidate entry
			if(DEBUG > 1) printf("    >  Ivalidate entry %d\n",unreachableDest);
			// invalidate entry and schedule deletion
			aodv_entry_invalidate(unreachableDest, unreachableDestSeqNb+1, n_flag);
			// go through all listOfPrecursors and remove the unreachable 
			// destination from them: this is done via reverseListOfPrecursors
			if(DEBUG > 1 ) printf("      > Remove unreachable node from all the precursor lists\n");
			aodv_listOfPrecursors_node_remove(unreachableDest);
			// add the new unreachable destination to the RERR packet
			if(DEBUG > 1 ) printf("      > Add destination to RERR packet\n");
			// add destination to the list of new unreachable destinations 
			aodv_listOfUnreachableDest_insert(newErrorStructPtr, unreachableDest, unreachableDestSeqNb);
			// print entry
			if(DEBUG) aodv_entryPtr_print(indexEntryError);
			}
		else
			if(DEBUG > 1) printf("      > skip entry %d\n",indexEntryError->dest);
		
		object=object->next;
		}
	}

// if list of unreachable destinations is not empty, then 
// generate a RERR packet.
if(getFifoSize(newErrorStructPtr->listOfUnreachableDest)>0)
	{
	// Create RERR packet
	rerr_pk_ptr = op_pk_create_fmt("AODV_RERR");
	// Set N field value
	op_pk_nfd_set(rerr_pk_ptr,"N", n_flag);
	// Set the error structure and add it to packet
	newErrorStructPtr->destCount = getFifoSize(newErrorStructPtr->listOfUnreachableDest);
	op_pk_nfd_set(rerr_pk_ptr,"ErrorContent",newErrorStructPtr,op_prg_mem_copy_create,op_prg_mem_free,sizeof(ErrorContentStruct));
	if(DEBUG > 1) printf("      > RERR packet has been generated\n");
	// Send to mac Layer 
	aodv_pk_send_to_mac_layer(rerr_pk_ptr, BROADCAST);
	}
else
	{
	if(DEBUG > 1) printf("      > No unreachable destination caused by link breakage\n");
	}
	
if(DEBUG > 1) printf("    - Function aodv_rerr_pk_generate done !\n");
FOUT;
}

/*****************************************************************/
/************************ handle link breakage  ******************/
/*****************************************************************/

void 
aodv_link_repair_attempt(int dest, int ttl_src)
{

// vars
RoutingTableEntry* entryPtr;
//int                TTL_REQUEST;
FIN(aodv_link_repair_attempt(int dest, int ttl_src));
/*
/* This function checks whether the lost link
/* is repairable or not. If so, it generates a 
/* RREQ with the appropriate TTL. Else, it 
/* generates a RERR for the lost destination.
*/

// init vars
entryPtr =   aodv_routingTable_entry_get(dest);

//begin
if(DEBUG > 1 ) printf("    - Function aodv_link_repair_attempt(destination %d)\n", dest);

if(entryPtr != OPC_NIL)
	{
	// reset request sent repository
	if(RequestSent[dest].status != WAITING_FOR_REPAIR)
		{
		if(RequestSent[dest].status != OFF)
			aodv_requestSent_repository_reset(dest);
		

		// check if destination is no farther than MAX_REPAIR_TTL
		if(entryPtr->lastHopCount <(MAX_REPAIR_TTL+1))
			{
			if(DEBUG > 1 ) printf("    > Link is repairable: mark entry UNDER_REPAIR\n");
			// Set entry status to "UNDER_REPAIR"
			entryPtr->status = UNDER_REPAIR;
			// print entry
			if(DEBUG) aodv_entryPtr_print(entryPtr);
			// Update the RequestSent repository:
			// Compute the TTL value to use
			RequestSent[dest].ttl = max(MIN_REPAIR_TTL, ttl_src)+LOCAL_ADD_TTL; 
			// generate RREQ
			if(DEBUG > 1 ) printf("    > Generating RREQ\n");			
			aodv_rreq_pk_generate(dest,WAITING_FOR_REPAIR);
			}
		else
			{
			// generate RERR packet for the lost destination
			if(DEBUG > 1 ) printf("    > Node is too far: generating RERR\n");
			aodv_rerr_pk_generate(dest,0);
			}
		}
	else
		if(DEBUG > 1 ) printf("    > Link already under repair: waiting for reply\n");

	}
FOUT;
}





/***********************************************************/
/********* aodv_listOfPrecursors_node_remove  **************/
/***********************************************************/
void 
aodv_listOfPrecursors_node_remove(int precursor)
	{
	// var
	int*                 destPtr;
	int                  dest,size,t;
	int*                 precursorPtr;
	RoutingTableEntry*   entryPtr;
	sFifo*               listOfDestination;
	FIN(aodv_listOfPrecursors_node_remove(int precursor));
	/*
	/* When a node (precursor) is no longer available, 
	/* the current function is called to remove it from
	/* any precursor list within the routing table. To 
	/* do so, the function reads the entry corresponding
	/* to the node "precursor" within the 
	/* reversePrecursorList and gets the list of entries
	/* where "precursor" is a precursor. It then goes 
	/* through these entries and removes the lost node
	/* form their precursor lists.
	*/

	// begin
	if(DEBUG > 3 ) printf("      - Function aodv_listOfPrecursors_node_remove(): precursor = %d\n",  precursor);
	
	// check if the unreachableDestination has an entry in the reversePrecursorList
	listOfDestination= (sFifo*) readInFifoMultiplex(reverseListOfPrecursors, precursor);
	if(listOfDestination != OPC_NIL)
		{
		size= getFifoSize(listOfDestination);
		if(DEBUG > 3 ) printf("      > Node is at least on one node\'s precursor list: total = %d\n",size);
		for(t=0;t<size;t++)
			{
			destPtr=getInFifo(listOfDestination);
			dest= (*destPtr); // PRECURSOR is present in DEST routing table entry
			if(DEBUG > 3 ) printf("         - on entry# %d... ", dest);
			op_prg_mem_free(destPtr);
			// the entry is read
			entryPtr= aodv_routingTable_entry_get( dest);
			if(entryPtr != OPC_NIL)
				{
				// the PRECURSOR entry is extracted from the list of precursors
				precursorPtr= (int*) getInFifoMultiplex(
							entryPtr->listOfPrecursors,precursor);
				op_prg_mem_free(precursorPtr);
				}
			else
				{
				if(DEBUG > 3 ) printf("         - Entry# %d does not exist", dest);
				}
			}
		}
	else
		if(DEBUG > 3 ) printf("      > Node was not found on any list of precursors !\n");
	if(DEBUG > 3 ) printf("\n      - Function aodv_listOfPrecursors_node_remove()... done !\n");
	FOUT;
	}


/////////////////////////////////////////////////////////////////
/******************** update list of precursors ****************/
/////////////////////////////////////////////////////////////////

void 
aodv_listOfPrecursors_node_put(RoutingTableEntry* forwardEntryPtr,int previousHop)
	{
	int      *int_ptr,
	         *previousHopPtr;
	FIN(aodv_listOfPrecursors_node_put(RoutingTableEntry* forwardEntryPtr,int previousHop));
	/*
	/* Add the node "previousHop" to the precursor list of 
	/* the given entry. 
	/* Note that node is put in subqueue whose index equals
	/* to its MAC address (previousHop). This makes it easier
	/* to remove the node if it lost.
	*/
	
	// if listOfPrecursors not initialized, then initialize it
	if(forwardEntryPtr->listOfPrecursors == OPC_NIL)
		{	
		forwardEntryPtr->listOfPrecursors = newFifo();
		}
	
	//check if node is already in
	int_ptr= (int*) readInFifoMultiplex(forwardEntryPtr->listOfPrecursors, 	previousHop); 
	if(int_ptr != OPC_NIL)
		{
		if(DEBUG > 3 ) printf("      > Precursor is already on the list !\n");
		}
	 else
		{
		if(DEBUG > 3 ) printf("      > Precursor does not exist, function has to add it!\n");
		// create a pointer to the node's address value
		previousHopPtr = (int*) op_prg_mem_alloc(sizeof(int));
		*previousHopPtr= previousHop;
		// add node to the precursol list
		if(DEBUG > 3 ) printf("      > Adding precursor %d\n",previousHop);
		putInFifoMultiplex(forwardEntryPtr->listOfPrecursors,previousHopPtr, previousHop);
		if(DEBUG > 3 ) printf("      > Previous Hop (%d) should have been added to precursor list of entry# %d\n", previousHop, forwardEntryPtr->dest);
		// update reverse list of precursors
		aodv_reverseListOfPrecursors_update(previousHop,forwardEntryPtr->dest);
		}
	 FOUT;
	}
	


/////////////////////////////////////////////////////////////////
/*********************** HANDLE HELLO MESSAGE ******************/
/////////////////////////////////////////////////////////////////
void 
aodv_hello_msg_receive(Packet* rrep_pk_ptr)
{
// var
int                 source, dest, hopCount;
int                 destSeqNb, previousHop;
double              lifetime;
//RoutingTableEntry * newEntryPtr, * forwardEntryPtr;
RoutingTableEntry  * forwardEntryPtr;
Boolean             request_was_pending = OPC_FALSE;
FIN(aodv_hello_msg_receive(Packet* rrep_pk_ptr));
/*
/* This routine processes the upcoming hello message.
*/
// begin

// packet read
op_pk_nfd_get (rrep_pk_ptr,"PreviousHop",&previousHop);
op_pk_nfd_get (rrep_pk_ptr,"SRC",&source);
op_pk_nfd_get (rrep_pk_ptr,"DEST",&dest);
op_pk_nfd_get (rrep_pk_ptr,"DestSeqNb",&destSeqNb);
// N.B. hop count field is incremented by 1at reception
op_pk_nfd_get (rrep_pk_ptr,"HopCount",&hopCount);
op_pk_nfd_set (rrep_pk_ptr,"HopCount",hopCount+1);
op_pk_nfd_get (rrep_pk_ptr,"HopCount",&hopCount);
op_pk_nfd_get (rrep_pk_ptr,"Lifetime",&lifetime);

if(DEBUG > 3) printf("    - Function aodv_hello_msg_receive(from node %d)\n",source);

// check if entry already exists
forwardEntryPtr=aodv_routingTable_entry_get( dest);
// if entry does not exist: create it and add it to table (with default values)
if(forwardEntryPtr == OPC_NIL)
	{
	// entry does not exist
	if(DEBUG > 1 ) printf("      > Entry does not exist\n");
	forwardEntryPtr=aodv_entry_create_new();
	forwardEntryPtr->listOfPrecursors=newFifo();
	forwardEntryPtr->dest = source;
	// add entry to routing table
	aodv_routingTable_entryPtr_put(forwardEntryPtr);
	}



if(aodv_entryPtr_hopCount_get(forwardEntryPtr) == INFINITY) 
	{
	// entry is either invalid or under repair or newly created
	if(DEBUG > 1 ) printf("      > Node creates entry\n");
	// Set entry values
	forwardEntryPtr->dest=dest;
	forwardEntryPtr->destSeqNb=destSeqNb;
	forwardEntryPtr->hopCount=hopCount;
	forwardEntryPtr->nextHop=previousHop;
	// check whether a request is pending
	if(RequestSent[dest].status != OFF)
		{
		if(DEBUG > 1 ) printf("      > A request was running for the newly created entry\n");
		// update requestSent repository
		aodv_requestSent_repository_reset(dest);
		// set serve buffer flag
		request_was_pending = OPC_TRUE;
		}
	}
else  // entry is active
	{
	if(DEBUG > 1 ) printf("      > Active entry already exist\n");
	// forward entry already exists
	if((destSeqNb > forwardEntryPtr->destSeqNb) ||((destSeqNb==forwardEntryPtr->destSeqNb)&&(hopCount < forwardEntryPtr->hopCount)))
		{
		// node updates his entry for the DESTINATION node only if (greater destSeqNb) or (same seqNbs but smaller hopCount)
		if(DEBUG > 1 ) printf("      > Node updates its entry\n");
		
		forwardEntryPtr->destSeqNb=destSeqNb;
		forwardEntryPtr->lastHopCount=forwardEntryPtr->hopCount;
		forwardEntryPtr->hopCount=hopCount;
		forwardEntryPtr->nextHop=previousHop;
		} /* end of if entry has to be udated */
	else
		if(DEBUG > 1 ) printf("      > Entry does not need to be updated\n");
	} /* end of if forward entry is active */


// Timeout always updated
if((op_sim_time()+lifetime) > forwardEntryPtr->expirationTime)
	{
	aodv_entry_expirationTime_set(dest,op_sim_time()+lifetime);
	}
else
	if(DEBUG > 1) printf("      > Entry timeout does not need to be updated\n");

// set status to active
forwardEntryPtr->status = ACTIVE;

// destroy packet
op_pk_destroy(rrep_pk_ptr);
// serve buffer if needed
if( request_was_pending == OPC_TRUE)
	{
	// serve buffer
	aodv_buffer_serve(dest);
	}

if(DEBUG > 1 ) printf("    - Function handleHelloMsg()  done\n");
FOUT;

} /* end of handleHelloMsg */

/////////////////////////////////////////////////////////////////
/*********************** HANDLE REPLY **************************/
/////////////////////////////////////////////////////////////////

void  
aodv_rrep_pk_receive(Packet* rrep_pk_ptr)
{
// var
Boolean    forward_entry_created_or_updated = OPC_FALSE;
int        source, dest, destSeqNb, hopCount;
//Ici*       ici_ptr;
FIN(aodv_rrep_pk_receive(Packet* rrep_pk_ptr));
/*
/* Receive RREP and decide whether to update or create
/* the corresponding entry and whether to forward or 
/* discard the received RREP.
*/

// begin
if(DEBUG > 1 ) printf("    - Function aodv_rrep_pk_receive()\n");
// packet read
op_pk_nfd_get (rrep_pk_ptr,"HopCount",&hopCount);
op_pk_nfd_set (rrep_pk_ptr,"HopCount",hopCount+1);

op_pk_nfd_get (rrep_pk_ptr,"SRC",&source);
op_pk_nfd_get (rrep_pk_ptr,"DEST",&dest);
op_pk_nfd_get (rrep_pk_ptr,"DestSeqNb",&destSeqNb);
op_pk_nfd_get (rrep_pk_ptr,"HopCount",&hopCount);

// nodes checks compares his own destSeqNb for this destination to the destSeqNb in the RREP packet
if(aodv_entry_destSeqNb_get(dest) <= destSeqNb)
	{
	if(DEBUG > 1) printf("      > Reply is fresh enough: node updates its routingTable\n");
  	forward_entry_created_or_updated = 	aodv_entry_update_or_create_from_rrep(rrep_pk_ptr);
	}
// if node's relay and a forward route's been updated or created, node forwards RREP
if((node_addr != source) && (forward_entry_created_or_updated == OPC_TRUE))
	{
	// node checks his routing table for a reverse entry 
	// if a route's available, node updates its reverse entry (list of precursor) and forwards rrep to nextHop
	if ((aodv_entry_nextHop_get(source) != NOT_VALID) && (aodv_entry_nextHop_get(source) != NON_EXISTENT))
		{
		// Packet has a reverse entry
		// add next Hop Toward Source to the precursor list of the forward entry
		aodv_listOfPrecursors_node_put(aodv_routingTable_entry_get(dest), aodv_entry_nextHop_get(source));
		// node forwards RREP
		aodv_rrep_pk_forward(rrep_pk_ptr);
		}
	else
		{
		// no reverse entry was found
		if(DEBUG > 1 ) printf("      > ERROR @ node %d: No reverse entry was found to forward RREP!\n", node_addr);
		}	 
	}

// if a forward entry's been created or updated, node schedule interruption to serve data buffer
if(forward_entry_created_or_updated == OPC_TRUE)
	{
	if(DEBUG > 1 ) printf("      > Serve the buffer\n");
	// serve buffer 
	aodv_buffer_serve(dest);
	}
if(DEBUG > 1 ) printf("      > Print forward entry if any available\n");
if(DEBUG > 1 ) aodv_entry_print(dest);
if(DEBUG > 1 ) printf("      > Print reverse entry if any available\n");
if(DEBUG > 1 ) aodv_entry_print(source);
if(DEBUG > 1 ) printf("    - Function aodv_rrep_pk_receive() done\n");
FOUT;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_rrep_pk_forward(Packet *rrep_pk_ptr)                       //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void 
aodv_rrep_pk_forward(Packet *rrep_pk_ptr)
	{
	// vars
	int source;
	
	/*
	/* Unicast the RREP to next hop along 
	/* the reverse path (next hop toward
	/* the source that generate the RREQ.
	*/
	FIN(aodv_rrep_pk_forward(Packet *rrep_pk_ptr));
	// init
	op_pk_nfd_get (rrep_pk_ptr,"SRC",&source);
	// send to mac layer
	if(DEBUG > 1 ) printf("    > Forward RREP to source node [next hop is %d]\n",aodv_entry_nextHop_get(source));
	aodv_pk_send_to_mac_layer(rrep_pk_ptr,aodv_entry_nextHop_get(source));
	// refresh reverse entry's timeout
	aodv_entry_expirationTime_set(source, op_sim_time()+ ACTIVE_ROUTE_TIMEOUT);
	FOUT;
	}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Boolean aodv_entry_update_or_create_from_rrep(Packet *rrep_pk_ptr)    //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

Boolean 
aodv_entry_update_or_create_from_rrep(Packet* rrep_pk_ptr)
	{
	// vars
	Boolean              result = OPC_FALSE;
	RoutingTableEntry *  forwardEntryPtr;
	int                  source, dest;
	FIN(aodv_entry_update_or_create_from_rrep(Packet* rrep_pk_ptr));
	/*
	/* when a RREP packet is received, a node either
	/* creates or updates the corresponding entry in 
	/* its routing table or simply does nothing.
	/* This function returns the value OPC_TRUE is a
	/* routing table entry was created or updated.
	/* 
	*/
	
	// init
	op_pk_nfd_get (rrep_pk_ptr,"DEST",&dest);
	// begin	
	if(aodv_routingTable_entry_get(dest) == OPC_NIL)
		{
		//node creates a new entry
		result = aodv_entry_create_from_rrep(rrep_pk_ptr);
		}
	else
		{
		forwardEntryPtr = aodv_routingTable_entry_get(dest);
		if(forwardEntryPtr->status == UNDER_REPAIR)
			{
			// node checks if entry is repairable
			if(DEBUG > 1 ) printf("      > Attempt to repair link from RREP\n");			
			result = aodv_entry_repair_from_rrep(rrep_pk_ptr);				
			}
		else
			{
			//node updates entry			
			result = aodv_entry_update_from_rrep(rrep_pk_ptr);
			}
		}

	// if reply's reached its destination, then node destroys it
	// node also updates its RequestSent repository
	op_pk_nfd_get (rrep_pk_ptr,"SRC",&source);
	if(node_addr == source)
		{
		// destroy rrep
		if(DEBUG > 1 ) printf("      > Reply has reached its destination: node destroys it.\n");
		op_pk_destroy(rrep_pk_ptr);
		// reset RequestSent repository
		if(RequestSent[dest].status != OFF)
			aodv_requestSent_repository_reset(dest);
		}

	FRET(result);
	}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Boolean aodv_entry_repair_from_rrep(Packet *rrep_pk_ptr)             //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

Boolean 
aodv_entry_repair_from_rrep(Packet* rrep_pk_ptr)
	{
	// vars
	RoutingTableEntry * entryPtr;
	Boolean             result = OPC_TRUE;
	int                 dest,destSeqNb,hopCount,
	                    previousHop, lastHopCount;
	double              lifetime;
	FIN(aodv_entry_repair_from_rrep(Packet* rrep_pk_ptr));
	/*
	/* This function when it is possible
	/* to repair an entry following the 
	/* reception of a RREP. In fact, when
	/* the alternative route is longer, node
	/* generates a RERR with the N flag set 
	/* to 1.
	*/

	// init
	op_pk_nfd_get (rrep_pk_ptr,"DEST",&dest);
	// read entry from routing table
	entryPtr = aodv_routingTable_entry_get(dest);
	// Store lastHopCount value
	lastHopCount = entryPtr->lastHopCount;
	// Read the lentgh of the new path
	op_pk_nfd_get (rrep_pk_ptr,"HopCount",&hopCount);
	// repair entry
	if(DEBUG > 1 ) printf("      > Repair entry from RREP\n");
	
	aodv_entryPtr_hopCount_set(entryPtr,hopCount);
	
	op_pk_nfd_get (rrep_pk_ptr,"DestSeqNb",&destSeqNb);	
	aodv_entryPtr_destSeqNb_set(entryPtr,destSeqNb);
		
	op_pk_nfd_get (rrep_pk_ptr,"PreviousHop",&previousHop);
	aodv_entryPtr_nextHop_set(entryPtr,previousHop);
		
	op_pk_nfd_get (rrep_pk_ptr,"Lifetime",&lifetime);
	aodv_entryPtr_expirationTime_set(entryPtr,op_sim_time()+lifetime);
	
	aodv_entryPtr_status_set(entryPtr,ACTIVE);
	// Print entry
	if(DEBUG) aodv_entryPtr_print(entryPtr);
	// if new hopCount is different than the former hopCount, generate a non-delete RERR 
	if(lastHopCount != hopCount) // maybe just in case >
		{
		if(DEBUG > 1 ) printf("      > New route hop count [%d] is different from the former [%d]: generating RERR with N flag set\n",hopCount,lastHopCount);
		// generate a RERR with N flag set to 1
		aodv_rerr_pk_generate(dest, 1);
		}
	else
		{
		if(DEBUG > 1 ) printf("      > Same hop count: no RERR generated\n");
		}
	// reset the Requestsent repository
	if(RequestSent[dest].status != OFF)
		aodv_requestSent_repository_reset(dest);

	FRET(result);
	}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_requestSent_repository_reset(int destination)              //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void
aodv_requestSent_repository_reset(int destination)
	{
	
	/*
	/* Reset requestSent repository
	*/
	FIN(aodv_requestSent_repository_reset(int destination));
	RequestSent[destination].ttl=TTL_START;
	RequestSent[destination].nbOfRetries = 0;
	RequestSent[destination].status = OFF;
    // Cancel interruption
	op_ev_cancel(RequestSent[destination].evt);
	FOUT;
	}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Boolean aodv_entry_update_from_rrep(Packet *rrep_pk_ptr)             //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

Boolean 
aodv_entry_update_from_rrep(Packet* rrep_pk_ptr)
	{
	// vars
	Boolean             result = OPC_FALSE;
	int                 dest,destSeqNb,hopCount, 
	                    previousHop;
	double              lifetime;
	RoutingTableEntry * entryPtr;
	FIN(aodv_entry_update_from_rrep(Packet* rrep_pk_ptr));
	/*
	/* When a node receives a RREP for a destination
	/* to which it already has an entry, this function
	/* is called in order to check whether node should
	/* update the corresponding entry or not. See 
	/* conditions bellow.
	*/
	
	// init
	// read RREP packet fields
	op_pk_nfd_get (rrep_pk_ptr,"DEST",&dest);
	op_pk_nfd_get (rrep_pk_ptr,"DestSeqNb",&destSeqNb);
	op_pk_nfd_get (rrep_pk_ptr,"HopCount",&hopCount);
	/// read entry from routing table
	entryPtr = aodv_routingTable_entry_get(dest);
	// Conditions for updates.
	// Node updates entry only if one of the following conditions is true
	// 1- New destSeqNb is greater than the current one
	// 2- same destSeqNb but smaller hopCount
	// 3- Route is no longer valid
	if((entryPtr->destSeqNb < destSeqNb) || ((entryPtr->destSeqNb == destSeqNb) && (entryPtr->hopCount > hopCount)) || (entryPtr->status == INVALID))
		{
		if(DEBUG > 1) printf("      > Node updates its forward entry\n");
		
		aodv_entryPtr_status_set(entryPtr,ACTIVE);
			
		aodv_entryPtr_destSeqNb_set(entryPtr,destSeqNb);

		aodv_entryPtr_hopCount_set(entryPtr,hopCount);
		
		op_pk_nfd_get (rrep_pk_ptr,"PreviousHop",&previousHop);
		aodv_entryPtr_nextHop_set(entryPtr,previousHop);
		
		op_pk_nfd_get (rrep_pk_ptr,"Lifetime",&lifetime);
		aodv_entryPtr_expirationTime_set(entryPtr,op_sim_time()+lifetime);

		// Set result to OPC_TRUE
		result = OPC_TRUE;
		}
	else
		if(DEBUG > 1) printf("      > No need to update the forward entry\n");
	
	FRET(result);
	}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// Boolean aodv_entry_create_from_rrep(Packet *rrep_pk_ptr)             //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

Boolean 
aodv_entry_create_from_rrep(Packet* rrep_pk_ptr)
	{

	Boolean             result = OPC_TRUE;
	RoutingTableEntry * newEntryPtr=aodv_entry_create_new();	
	int                  dest,destSeqNb,
	                    hopCount, previousHop;
	double              lifetime;
	FIN(aodv_entry_create_from_rrep(Packet* rrep_pk_ptr));
	/*
	/* Create a new entry from a RREP packet
	/* and add it to node's Routing Table.
	*/
	
	op_pk_nfd_get (rrep_pk_ptr,"DEST",&dest);
	
	// Copy different values from RREP to the new entry
	if(RequestSent[dest].status != OFF)
		aodv_requestSent_repository_reset(dest);
	
   
	aodv_entryPtr_destination_set(newEntryPtr,dest);

	op_pk_nfd_get (rrep_pk_ptr,"DestSeqNb",&destSeqNb);	
	aodv_entryPtr_destSeqNb_set(newEntryPtr,destSeqNb);

	op_pk_nfd_get (rrep_pk_ptr,"HopCount",&hopCount);
	aodv_entryPtr_hopCount_set(newEntryPtr,hopCount);
	
	op_pk_nfd_get (rrep_pk_ptr,"PreviousHop",&previousHop);
	aodv_entryPtr_nextHop_set(newEntryPtr,previousHop);

	op_pk_nfd_get (rrep_pk_ptr,"Lifetime",&lifetime);
	aodv_entryPtr_expirationTime_set(newEntryPtr,op_sim_time()+lifetime);
	newEntryPtr->listOfPrecursors=newFifo();
	
	// node adds entry to RoutingTable
	aodv_routingTable_entryPtr_put(newEntryPtr);
	if(DEBUG > 1 ) printf("      > Forward entry created and added to routing table.\n");
	
	FRET(result);
	}


/////////////////////////////////////////////////////////////////
/*********************** HANDLE ERROR **************************/
/////////////////////////////////////////////////////////////////

void 
aodv_rerr_pk_receive(Packet* rerr_pk_ptr)
{
// var
int                    previousHop,n_flag, unreachableDest,
                       unreachableDestSeqNb, 
					   new_rerr_generated = 0, i;
ErrorContentStruct    *errorStructPtr,
                      *newErrorStructPtr ;
Packet *               new_rerr_pk_ptr;
int                    retransmit_rerr_pk = 0;
int                    rerr_reached_source_node = 0;
FIN(aodv_rerr_pk_receive(Packet* rerr_pk_ptr));
/*
/* Receive RERR packets. Invalidate appropriate
/* entries. Update list of unreachable nodes.
/* Generate a new RERR packet if new list of 
/* unreachable nodes is not empty.
*/

// var init
newErrorStructPtr= (ErrorContentStruct*) op_prg_mem_alloc(sizeof(ErrorContentStruct));
newErrorStructPtr->listOfUnreachableDest=newFifo();
// Read RERR packet
op_pk_nfd_get (rerr_pk_ptr,"PreviousHop",&previousHop);
op_pk_nfd_access (rerr_pk_ptr,"ErrorContent",&errorStructPtr);
op_pk_nfd_get (rerr_pk_ptr,"N",&n_flag);


if(DEBUG > 1 ) printf("    - Function aodv_rerr_pk_receive()\n");
 
// For each unreachable dest
for(i=0; i< errorStructPtr->destCount; i++) 
	{
	// read the unreachable destination attributes (address + seq nb)
	unreachableDest = aodv_listOfUnreachableDest_dest_getFirst(errorStructPtr);
	unreachableDestSeqNb = aodv_listOfUnreachableDest_destSeqNb_getFirst(errorStructPtr);
	if(DEBUG > 1 ) printf("      > Unreachable dest = %d\n",unreachableDest);
	if(DEBUG > 1 ) aodv_entry_print(unreachableDest);
	// node checks if PreviousHop is the nextHop for this destination	
	if(aodv_entry_nextHop_get(unreachableDest) == previousHop)
		{
		if(DEBUG > 1 ) printf("      > Node which forwarded RERR is the nextHop for this destination: node has to invalidate it\n");
		// if precursor list is not empty, node generates a new RERR 
		if(aodv_entry_listOfPrecursors_is_empty(unreachableDest) == OPC_FALSE && n_flag == 0)
			{
			if(DEBUG > 1 ) printf("      > Precursor list is not empty: dest remains within the list of unreachable dest\n");
			// Set new_rerr_generated flag to 1
			new_rerr_generated =1;
			// add destination to the list of new unreachable destinations 
			aodv_listOfUnreachableDest_insert(newErrorStructPtr, unreachableDest, unreachableDestSeqNb);
			// Flushes destination's list of precursors
			if(DEBUG > 1 ) printf("      > Flush list of precursors\n");
			aodv_entry_listOfPrecursors_flush(unreachableDest);
			}
		else if (aodv_entry_listOfPrecursors_is_empty(unreachableDest) == OPC_TRUE && n_flag == 1)
			{
			if(DEBUG > 1 ) printf("      > Source node has been reached\n");
			 rerr_reached_source_node =1;
			 }
		
		else if (aodv_entry_listOfPrecursors_is_empty(unreachableDest) == OPC_FALSE && n_flag == 1)
			{
			if(DEBUG > 1 ) printf("      > Node should retransmit RERR\n");
			retransmit_rerr_pk =1;
			}
		if(n_flag == 0 || (n_flag == 1 && rerr_reached_source_node))
			{
			// node invalidates the entry
			if(DEBUG > 1 ) printf("      > Invalidate entry %d\n",unreachableDest);
			aodv_entry_invalidate(unreachableDest, unreachableDestSeqNb , n_flag);
			if(n_flag == 0)
				{
				// go through all listOfPrecursors and remove the unreachable 
				// destination from them
				if(DEBUG > 1 ) printf("      > Remove from precursor lists\n");
				aodv_listOfPrecursors_node_remove(unreachableDest);
				}
			//print entry
			if(DEBUG) aodv_entry_print(unreachableDest);
			}
		}
	else
		if(DEBUG > 1 ) printf("      > PreviousHop is not the next hop for this destination: skip to next unreachable error if any\n");
	 }

// All unreachable destinations have been processed
if(retransmit_rerr_pk)
	{
	if(DEBUG > 1 ) printf("      > Forward RERR packet\n");
	aodv_pk_send_to_mac_layer(rerr_pk_ptr, BROADCAST);
	}
else if(rerr_reached_source_node)
	{
    op_pk_destroy(rerr_pk_ptr);
	if(RequestSent[unreachableDest].status != OFF)
		aodv_requestSent_repository_reset(unreachableDest);
	if(DEBUG > 1 ) printf("      > Generate RREQ\n");
	aodv_rreq_pk_generate(unreachableDest, WAITING_FOR_REPLY);
	}
else
	op_pk_destroy(rerr_pk_ptr);
if(DEBUG > 1 ) printf("      > RERR destroyed\n");

// check if new rerr should be generated or not 
if(new_rerr_generated) 
	{
	if(DEBUG > 1 ) printf("      > Generating new RERR\n");
	// Set destCount field to the number of unreachable destinations
	newErrorStructPtr->destCount = getFifoSize(newErrorStructPtr->listOfUnreachableDest);
	// Create a new RERR packet
	new_rerr_pk_ptr= op_pk_create_fmt("AODV_RERR");
	op_pk_nfd_set(new_rerr_pk_ptr,"ErrorContent",newErrorStructPtr,op_prg_mem_copy_create,op_prg_mem_free, sizeof(ErrorContentStruct));
	op_pk_nfd_set(new_rerr_pk_ptr,"N",n_flag);
	// Send to MAC layer interface
	if(DEBUG > 1 ) printf("      > Send to mac\n");
	aodv_pk_send_to_mac_layer(new_rerr_pk_ptr,BROADCAST);
	}
else
	{
	if(DEBUG > 1 ) printf("      > No more unreachable destinations\n");
	}


if(DEBUG > 1 ) printf("    - Function aodv_rerr_pk_receive() done\n");
FOUT;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// void aodv_listOfUnreachableDest_insert(ErrorContentStruct *errorStructPtr,int dest, int destSeqNb) //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

void aodv_listOfUnreachableDest_insert(ErrorContentStruct *newErrorStructPtr, int unreachableDest, int unreachableDestSeqNb)
	{
	// var		
	UnreachableDestStruct *newUnreachableDestStructPtr = (UnreachableDestStruct*)  op_prg_mem_alloc(sizeof(UnreachableDestStruct));
	FIN(aodv_listOfUnreachableDest_insert(ErrorContentStruct *newErrorStructPtr, int unreachableDest, int unreachableDestSeqNb));
	/*
	/* Insert a new unreachableDestStructure 
	/* (Destination Address + Destination Seq.
	/* Nb. ) into the list of Unreachable 
	/* Destinations
	*/
	
	// begin
	// set destination
	(*newUnreachableDestStructPtr).dest = unreachableDest;
	// set destination seq number
	(*newUnreachableDestStructPtr).destSeqNb = unreachableDestSeqNb;
	// add to error packet
	putInFifo( newErrorStructPtr->listOfUnreachableDest, newUnreachableDestStructPtr);
	FOUT;
	}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// int aodv_listOfUnreachableDest_dest_getFirst(ErrorContentStruct *errorStructPtr) //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

int aodv_listOfUnreachableDest_dest_getFirst(ErrorContentStruct *errorStructPtr)
	{
	
	/*
	/* Read the first unreachable destination 
	/* Address from the RERR packet.
	*/
	FIN(aodv_listOfUnreachableDest_dest_getFirst(ErrorContentStruct *errorStructPtr));
	FRET((*((UnreachableDestStruct*) readInFifo(errorStructPtr->listOfUnreachableDest))).dest);
	}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// int aodv_listOfUnreachableDest_destSeqNb_getFirst(ErrorContentStruct *errorStructPtr) //
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

int aodv_listOfUnreachableDest_destSeqNb_getFirst(ErrorContentStruct *errorStructPtr)
	{
	/*
	/* Read the first unreachable destination 
	/* Seq Nb. from the RERR packet.
	*/

	// vars
	// Read the first element from the list
	UnreachableDestStruct *unreachableDestStructPtr = (UnreachableDestStruct*) getInFifo(errorStructPtr->listOfUnreachableDest);
	// Store the dest seq nb value
	int destSeqNb = unreachableDestStructPtr->destSeqNb;
	FIN(aodv_listOfUnreachableDest_destSeqNb_getFirst(ErrorContentStruct *errorStructPtr));
	//re-put the structure at the end of the fifo
	putInFifo(errorStructPtr->listOfUnreachableDest,unreachableDestStructPtr);
	// Return the result
	FRET(destSeqNb);
	}

/***********************************************************/
/*************  aodv_entryPtr_print ************************/
/***********************************************************/
void 
aodv_entryPtr_print(RoutingTableEntry* entryPtr)
	{
	char status[256];
	FIN(aodv_entryPtr_print(RoutingTableEntry* entryPtr));
	
	/*
	/* Print a pointer to a routing table entry
	*/
	sprintf(status,"ACTIVE");
	
	if(entryPtr != OPC_NIL){
	printf("      /***************************************/\n");
	printf("      /***** Entry for [ %d ] @ node %d\n",entryPtr->dest,node_addr);
	printf("      /***************************************/\n");
	if(entryPtr->status == INVALID)
		sprintf(status, "INVALID");
	else if (entryPtr->status == UNDER_REPAIR)
		sprintf(status, "UNDER_REPAIR");
	printf("      /** Status         = %s\n",status);
	
	printf("      /** Next Hop       = %d\n",entryPtr->nextHop);
	printf("      /** DestSeqNb      = %d\n",entryPtr->destSeqNb);
	
	if(entryPtr->hopCount != INFINITY) 
		printf("      /** Hop Count      = %d\n",entryPtr->hopCount);
	else
		printf("      /** Hop Count      = INFINITY\n");
	
	if(entryPtr->lastHopCount != INFINITY)
		printf("      /** Last Hop Count = %d\n",entryPtr->lastHopCount);
	else
		printf("      /** Last Hop Count = INFINITY\n");

	printf("      /** ExpirationTime = %f\n",entryPtr->expirationTime);
	printf("      /** Current Time   = %f\n",op_sim_time());
	if(getFifoSize(entryPtr->listOfPrecursors)>0)
		{
		printf("      /********* list of precursors  *********/\n");
		printFifo(*(entryPtr->listOfPrecursors));
		}
	printf("      /***************************************/\n");	
	}
	FOUT;
}
/***********************************************************/
/******************* aodv_entry_print ****************************/
/***********************************************************/
void 
aodv_entry_print(int destination)
	{
	// vars
	RoutingTableEntry* entry;
	FIN(aodv_entry_print(int destination));
	/* 
	/* Print the routing table entry for a 
	/* given destination
	*/
	
	// read a pointer to the correpoding entry
	entry= aodv_routingTable_entry_get( destination);
	if(entry != OPC_NIL)
		aodv_entryPtr_print(entry);
	else
		printf("      > No entry for destination %d\n",destination);
	FOUT;
	}

/***********************************************************/
/*********************  PRINT PACKET ***********************/
/***********************************************************/

void 
aodv_pk_print(Packet* pk_ptr)
	{
	int                     ttl, n_flag, pkType,nextHop, 
	                        previousHop,src,dest, 
							destSeqNb, srcSeqNb, broadcastID,
							hopCount;
	double                  lifetime;
	//UnreachableDestStruct * unreachableDestStructPtr;
	ErrorContentStruct *    errorStructPtr;
	FIN(aodv_pk_print(Packet* pk_ptr));
	/* 
	/* Print the packet given in parameter
	*/
	
	// get the type of the packet
	op_pk_nfd_get(pk_ptr,"Type",&pkType);
	op_pk_nfd_get(pk_ptr,"NextHop",&nextHop);
	op_pk_nfd_get(pk_ptr,"PreviousHop",&previousHop);

	switch(pkType) {
		case DATA_PACKET_TYPE: // DATA	
			{			
  			op_pk_nfd_get(pk_ptr,"SRC",&src);
			op_pk_nfd_get(pk_ptr,"DEST",&dest);
			
			printf("      /******************************/\n");
			printf("      /********* DATA Packet ********/\n");
			printf("      /******************************/\n");
			printf("      /*  From  = %d\n",previousHop);
			printf("      /*  To    = %d\n",nextHop);						
			printf("      /------------------------------/\n");			
			printf("      /*  Source      = %d\n",src);
			printf("      /*  Destination = %d\n",dest);
			printf("      /******************************/\n");
			break;
			}
		case REQUEST_PACKET_TYPE: // RREQ received
			{
			op_pk_nfd_get(pk_ptr,"TTL",&ttl);			
			op_pk_nfd_get(pk_ptr,"SRC",&src);
			op_pk_nfd_get(pk_ptr,"DEST",&dest);
			op_pk_nfd_get(pk_ptr,"DestSeqNb",&destSeqNb);
			op_pk_nfd_get(pk_ptr,"BroadcastID",&broadcastID);
			op_pk_nfd_get(pk_ptr,"SrcSeqNb",&srcSeqNb);
			op_pk_nfd_get(pk_ptr,"HopCount",&hopCount);			
			
			 printf("     /******************************/\n");			
			 printf("     /******** REQUEST Packet ******/\n");
			 printf("     /******************************/\n");
			 printf("     /*  TTL   = %d\n",ttl);
			 printf("     /------------------------------/\n");
			 printf("     /*  From  = %d\n",previousHop);
			 printf("     /------------------------------/\n");
			 printf("     /*  Source      = %d\n",src);
			 printf("     /*  Destination = %d\n",dest);
			 printf("     /*  DestSeqNb   = %d\n",destSeqNb);
			 printf("     /*  HopCount    = %d\n",hopCount);
     		 printf("     /*  BroadcastID = %d\n",broadcastID);
			 printf("     /*  SrcSeqNb    = %d\n",srcSeqNb);
			 printf("     /******************************/\n");
			break;
			}
		
		case REPLY_PACKET_TYPE: // RREP received
			{
			op_pk_nfd_get(pk_ptr,"SRC",&src);
			op_pk_nfd_get(pk_ptr,"DEST",&dest);
			op_pk_nfd_get(pk_ptr,"DestSeqNb",&destSeqNb);
			op_pk_nfd_get(pk_ptr,"HopCount",&hopCount);
			op_pk_nfd_get(pk_ptr,"Lifetime",&lifetime);
			
			 printf("     /******************************/\n");			
			 if(nextHop > -1)
				 printf("     /********* REPLY Packet *******/\n");
			else
				 printf("     /*********** HELLO Msg ********/\n");
			 printf("     /******************************/\n");
			 printf("     /*  From  = %d\n",previousHop);
			 printf("     /*  To    = %d\n",nextHop);
			 printf("     /------------------------------/\n");			
			 printf("     /*  Source      = %d\n",src);
			 printf("     /*  Destination = %d\n",dest);
			 printf("     /*  DestSeqNb   = %d\n",destSeqNb);
			 printf("     /*  HopCount    = %d\n",hopCount);
			 printf("     /*  Lifetime    = %f\n",lifetime);
			 printf("     /******************************/\n");
			break;
			}
		case ERROR_PACKET_TYPE: // RERR received
			{
			 printf("     /******************************/\n");
			 printf("     /********* ERROR Packet *******/\n");
			 printf("     /******************************/\n");
			 printf("     /*  From  = %d\n",previousHop);
			 printf("     /------------------------------/\n");
			 op_pk_nfd_access(pk_ptr,"N",&n_flag);
			 printf("     /*  N     = %d\n",n_flag);
			 printf("     /------------------------------/\n");
			 op_pk_nfd_access(pk_ptr,"ErrorContent",&errorStructPtr);

			 aodv_unreachableDestList_print(*(errorStructPtr->listOfUnreachableDest));
			 printf("     /******************************/\n");
			 break;
			}
		}
	FOUT;
	}


/***********************************************************/	
/**************  aodv_unreachableDestList_print ************/
/***********************************************************/

void 
aodv_unreachableDestList_print(sFifo fifo)
{
// var
UnreachableDestStruct*  i;
int                     nbr;
sObject*               object;
FIN(aodv_unreachableDestList_print(sFifo fifo));
/*
/* Print the list of unreachable destinations
/* in a RERR packet.
*/
// Initiate cursor to the first position  
object=  fifo.firstObject;
// go through the list 
for(nbr=0;nbr<getFifoSize(&fifo);nbr++)
	{
	i=(UnreachableDestStruct*)(object->data);
	if(DEBUG > 1 ) printf("     /* Destination= %d || DestSeqNb= %d\n",i->dest,i->destSeqNb);
	object=object->next;
	}
FOUT;
}
	
/***********************************************************/	
/**************  printRoutingTable *************************/
/***********************************************************/
void 
aodv_routingTable_print()
	{
	//var
	int                  i;
	RoutingTableEntry*   entryPtr;
	sObject*             object;
	FIN(aodv_routingTable_print());
	/*
	/* Print the routingTable of the current node
	*/
	
	// Initiate the cursor to the first position 
	object=  routingTable->firstObject;
	
	//begin
	 printf("     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	 printf("     ~~~~~~~~~~~~~  ROUTING TABLE (node %d)  ~~~~~~~~~~~~~~~\n",node_addr);
	 printf("     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	// For each entry
	for(i=0;i<getFifoSize(routingTable);i++)
		{
		// read a pointer to the current entry
		entryPtr = (RoutingTableEntry*)(object->data);
		// print it
		aodv_entryPtr_print(entryPtr);
		// Skip to the next entry 
		object=object->next;
		}
	 printf("\n     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	 printf("     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n");
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

/***********************************************************/
/******************* print ack ici ***************************/
/***********************************************************/

void 
aodv_ack_print(int nextHop, int destination)
	{
	/*
	/* Print the ICI associated to an ACK Interruption
	*/
	FIN(aodv_ack_print(int nextHop, int destination));
	printf("       /***************************************/\n");
	printf("       /************ ACK EVENT ICI ************/\n");
	printf("       /***************************************/\n");								
	printf("       /*** Destination  = %d\n", nextHop);
	printf("       /*** FinalDest    = %d\n", destination);
	printf("       /***************************************/\n");
	FOUT;
	}

/////////////////////////////////////////////////////////////////
/************** is request fresh enough ************************/
/////////////////////////////////////////////////////////////////

Boolean 
aodv_rreq_pk_is_fresh_enough(Packet* rreq_pk_ptr)
	{
	// var
	Boolean     result = OPC_FALSE;
	int         source, dest, 
	            broadcastID;
	/*
	/* Check whether the BroadcastID of the request 
	/* is greater than the last seen. If so, RREQ is 
	/* automatically processed. Else, node checks
	/* whether RREQ packet was seen less than
	/* RECORD_BROADCAST_TIME seconds ago. If so, RREQ
	/* must be discarded. Else, it is processed.
	*/
	FIN(aodv_rreq_pk_is_fresh_enough(Packet* rreq_pk_ptr));
	// Read RREQ fields
	op_pk_nfd_get (rreq_pk_ptr,"BroadcastID",&broadcastID);
	op_pk_nfd_get (rreq_pk_ptr,"SRC",&source);
	op_pk_nfd_get (rreq_pk_ptr,"DEST",&dest); 

	if((RequestSeen[source][dest].broadcastID < broadcastID) ||
		((RequestSeen[source][dest].broadcastID==broadcastID) &&
			(op_sim_time()> RequestSeen[source][dest].expirationTime)))
		{
		result = OPC_TRUE;
		// update RequestSeen repository
		RequestSeen[source][dest].broadcastID=broadcastID;
		RequestSeen[source][dest].expirationTime=op_sim_time()+BROADCAST_RECORD_TIME;
		}	
	FRET(result);
	}

/////////////////////////////////////////////////////////////////
/************** is fresh enough entry available ****************/
/////////////////////////////////////////////////////////////////

Boolean 
aodv_fresh_enough_entry_is_available(int destination, int destSeqNb)
	{
	// var
	Boolean              result = OPC_FALSE;
	RoutingTableEntry *  entryPtr = aodv_routingTable_entry_get(destination);
	FIN(aodv_fresh_enough_entry_is_available(int destination, int destSeqNb));
	/*
	/* check whether a fresh enough (with regard to the given
	/* destination sequence number) active entry is available  
	/* within the routing table or not.
	*/
	
	// begin
	if((entryPtr != OPC_NIL) &&(entryPtr->hopCount != INFINITY) && (entryPtr->destSeqNb >= destSeqNb))
		{
		result = OPC_TRUE;
		}				
	FRET(result);
	}


/////////////////////////////////////////////////////////////////
/********* aodv_routingTable_entry_get *************************/
/////////////////////////////////////////////////////////////////

RoutingTableEntry *
aodv_routingTable_entry_get(int destination)
	{
	/*
	/* Return a pointer to the entry 
	/* corresponding to the requested 
	/* destination.
	*/
	FIN(aodv_routingTable_entry_get(int destination));
	FRET((RoutingTableEntry*) readInFifoMultiplex(routingTable, destination));
	}

/////////////////////////////////////////////////////////////////
/********* aodv_routingTable_entry_delete *************************/
/////////////////////////////////////////////////////////////////

void 
aodv_routingTable_entry_delete(int dest)
	{
	/*
	/* Delete the corresponding entry
	/* from the routingTable
	*/
	RoutingTableEntry *entryPtr;
	FIN(aodv_routingTable_entry_delete(int dest));
	entryPtr=(RoutingTableEntry*) getInFifoMultiplex(routingTable, dest);
	
	if(entryPtr != OPC_NIL)
		{
		op_prg_mem_free(entryPtr->listOfPrecursors);
		op_prg_mem_free(entryPtr);
		if (DEBUG) printf("    - Entry %d has been deleted\n", dest);
		}  
	FOUT;
	}

/////////////////////////////////////////////////////////////////
/********* aodv_routingTable_entryPtr_put *************************/
/////////////////////////////////////////////////////////////////

void 
aodv_routingTable_entryPtr_put(RoutingTableEntry *newEntryPtr)
	{
	/*
	/* Add the entry to the routingTable
	*/
	FIN(aodv_routingTable_entryPtr_put(RoutingTableEntry *newEntryPtr));
	putInFifoMultiplex(routingTable, newEntryPtr, newEntryPtr->dest);
	FOUT;
	}
/////////////////////////////////////////////////////////////////
/************** get active entry  ****************/
/////////////////////////////////////////////////////////////////

int 
aodv_entry_nextHop_get(int destination)
	{
	/*
	/* Get the next hop toward 
	/* a given destination. If 
	/* entry does not exist, 
	/* return NON_EXISTENT value.
	/* If not valid, return 
	/* NOT_VALID value. Else, return
	/* next hop address.
	*/
	// var
	int nextHop = NOT_VALID;
	RoutingTableEntry *entryPtr;
	FIN(aodv_entry_nextHop_get(int destination));
	// begin	
	entryPtr =aodv_routingTable_entry_get( destination);

	if(entryPtr == OPC_NIL)
		nextHop = NON_EXISTENT;
	else if(entryPtr->hopCount > -1)
		nextHop = entryPtr->nextHop;

	FRET(nextHop);
	}

/////////////////////////////////////////////////////////////////
/************** aodv_entry_expirationTime_get   ****************/
/////////////////////////////////////////////////////////////////

double 
aodv_entry_expirationTime_get(int destination)
	{
	/*
	/* Return the expiration time of 
	/* a given entry.
	*/
	// var
	double result = 0;
	RoutingTableEntry * entryPtr;
	FIN(aodv_entry_expirationTime_get(int destination));
	entryPtr = aodv_routingTable_entry_get(destination);
	// begin	
	if(entryPtr != OPC_NIL)
		result = entryPtr->expirationTime;

	FRET(result);
	}

/////////////////////////////////////////////////////////////////
/************** aodv_entry_expirationTime_set   ****************/
/////////////////////////////////////////////////////////////////

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

/////////////////////////////////////////////////////////////////
/************** aodv_entryPtr_expirationTime_set   ****************/
/////////////////////////////////////////////////////////////////

void 
aodv_entryPtr_expirationTime_set(RoutingTableEntry *entryPtr, double expirationTime)
	{
	/*
	/* Set the expiration time of a given pointer
	/* to entry.
	*/
	FIN(aodv_entryPtr_expirationTime_set(RoutingTableEntry *entryPtr, double expirationTime));
	if(DEBUG > 1) printf("      - Refresh timeout for entry %d (+ %f)\n", entryPtr->dest, (expirationTime-op_sim_time()));
	//if(DEBUG > 1) ("      - Time is %d\n", (int) op_sim_time());
	// copy the expirationTime value to the lastExpirationTime field
	entryPtr->lastExpirationTime =  entryPtr->expirationTime;
	// Copy the new expiration time the expirationTime field
	entryPtr->expirationTime = expirationTime;
	// Schedule new interruption for entry invalidation
	aodv_entryPtr_expirationInterrupt_schedule(entryPtr);
	if(DEBUG > 3) aodv_entry_print(entryPtr->dest);
	FOUT;
	}

/////////////////////////////////////////////////////////////////
/************** aodv_entryPtr_hopCount_set   ****************/
/////////////////////////////////////////////////////////////////

void 
aodv_entryPtr_hopCount_set(RoutingTableEntry *entryPtr, int newHopCount)
	{
	
	/*
	/* Copy the last hop count value, and set
	/* the new one to newhopCount.
	*/
	FIN(aodv_entryPtr_hopCount_set(RoutingTableEntry *entryPtr, int newHopCount));
	if(entryPtr->hopCount != INFINITY)
		entryPtr->lastHopCount=entryPtr->hopCount;

	entryPtr->hopCount=newHopCount;	
	FOUT;
	}
/////////////////////////////////////////////////////////////////
/************** aodv_entryPtr_hopCount_get   ****************/
/////////////////////////////////////////////////////////////////

int 
aodv_entryPtr_hopCount_get(RoutingTableEntry *entryPtr)
	{
	
	/*
	/* Return the hopCount value of a given 
	/* pointer to entry.
	*/
	FIN(aodv_entryPtr_hopCount_get(RoutingTableEntry *entryPtr));
	FRET(entryPtr->hopCount);
	}
/////////////////////////////////////////////////////////////////
/************** aodv_entry_hopCount_get   ****************/
/////////////////////////////////////////////////////////////////

int 
aodv_entry_hopCount_get(int dest)
	{
	int result =0;
	/*
	/* Return the hopCount value of a given 
	/* entry. If entry does not exist return 0
	*/
	FIN(aodv_entry_hopCount_get(int dest));
	if (aodv_routingTable_entry_get(dest) != OPC_NIL)
		result = aodv_entryPtr_hopCount_get(aodv_routingTable_entry_get(dest));

	FRET(result);
	}


/***********************************************************/
/****** aodv_entryPtr_expirationInterrupt_schedule() **********/
/***********************************************************/
void 
aodv_entryPtr_expirationInterrupt_schedule(RoutingTableEntry* entry)
	{
	
	/*
	/* Remove the current expiration event associated 
	/* to the entryexpirationTime and schedule a new 
	/* expiration event for the next expirationTime.
	*/
	FIN(aodv_entryPtr_expirationInterrupt_schedule(RoutingTableEntry* entry));
	if(DEBUG > 2) printf("      - Function aodv_entryPtr_expirationInterrupt_schedule(): entry= %d \n", entry->dest);

	// Cancel the previous interruption 
	aodv_entryPtr_expirationInterrupt_cancel(entry);
	// schedule new interruption
	entry->expiration_evt = op_intrpt_schedule_self(entry->expirationTime,entry->dest); 
	if(DEBUG > 2 ) printf("      - Function aodv_entryPtr_expirationInterrupt_schedule()... done !\n");
	FOUT;
	}

/////////////////////////////////////////////////////////////////
/********** aodv_entry_invalidate   ****************************/
/////////////////////////////////////////////////////////////////

void 
aodv_entry_invalidate(int dest, int destSeqNb, int n_flag)
	{
	// read entry from routingTable
	RoutingTableEntry *entryPtr = aodv_routingTable_entry_get(dest);
	FIN(aodv_entry_invalidate(int dest, int destSeqNb, int n_flag));
	/*
	/* Invalidate entry for the given destination (dest).
	/* Set hopCount to infinity. Set status to the appropriate
	/* value (INVALID or UNDER_REPAIR) and schedule deletion time
	/* in the former case (stauts = INVALID).
	*/
	
	if(entryPtr != OPC_NIL)
		{
		// set HopCount to infinity
		if(aodv_entryPtr_hopCount_get(entryPtr) != INFINITY)
			aodv_entryPtr_hopCount_set(entryPtr,INFINITY);
		// set destSeqNb
		aodv_entryPtr_destSeqNb_set(entryPtr,destSeqNb);
		// if n_flag set, node should not delete entry 
		// else, node should delete entry 
		if (n_flag)
			{
			// cancel expiration time
			entryPtr->lastExpirationTime = entryPtr->expirationTime;
			entryPtr->expirationTime = op_sim_time();
			aodv_entryPtr_expirationInterrupt_cancel(entryPtr);
			}
		else
			{
			// schedule entry deletion time
			aodv_entryPtr_expirationTime_set(entryPtr, op_sim_time()+DELETE_PERIOD);
			}	
		// set status to INVALID
		aodv_entryPtr_status_set(entryPtr, INVALID);
		
		}
	FOUT;
	}

/////////////////////////////////////////////////////////////////
/************ aodv_entry_listOfPrecursors_is_empty **************/
/////////////////////////////////////////////////////////////////

Boolean 
aodv_entry_listOfPrecursors_is_empty(int dest)
	{
	Boolean result = OPC_TRUE;
	RoutingTableEntry *entryPtr = aodv_routingTable_entry_get(dest);
	FIN(aodv_entry_listOfPrecursors_is_empty(int dest));
	/*
	/* Check whether the precursor list of the given 
	/* entry is empty or not.
	*/
	if(getFifoSize(entryPtr->listOfPrecursors) > 0)
		result= OPC_FALSE;
	FRET(result);
	}

/////////////////////////////////////////////////////////////////
/************ aodv_entry_listOfPrecursors_flush **************/
/////////////////////////////////////////////////////////////////

void 
aodv_entry_listOfPrecursors_flush(int dest)
	{
	RoutingTableEntry *entryPtr = aodv_routingTable_entry_get(dest);
	FIN(aodv_entry_listOfPrecursors_flush(int dest));
	/*
	/* Flush the precursor list of the given
	/* entry.
	*/
	
	// free the former list
	op_prg_mem_free(entryPtr->listOfPrecursors);
	// initialize a new one
	entryPtr->listOfPrecursors= newFifo();
	FOUT;	
	}

/////////////////////////////////////////////////////////////////
/****************** aodv_entry_destSeqNb_get     ***************/
/////////////////////////////////////////////////////////////////
int 
aodv_entry_destSeqNb_get(int destination) 
{
int result = 0;
FIN(aodv_entry_destSeqNb_get(int destination));
/*
/* Return the destination sequence number for
/* the given entry.
*/

if(aodv_routingTable_entry_get(destination) != OPC_NIL)
	{
	result = (*aodv_routingTable_entry_get(destination)).destSeqNb; 
	}
FRET(result);
}

//////////////////////////////////////////////////////////////////
/****************** aodv_entryPtr_destSeqNb_set   ***************/
//////////////////////////////////////////////////////////////////
void 
aodv_entryPtr_destSeqNb_set(RoutingTableEntry *entryPtr,int destSeqNb)
	{
	/*
	/* Return the destination sequence number for
	/* the given pointer to entry.
	*/
	FIN(aodv_entryPtr_destSeqNb_set(RoutingTableEntry *entryPtr,int destSeqNb));
	entryPtr->destSeqNb = destSeqNb;
	FOUT;
	}
/////////////////////////////////////////////////////////////////
/*************** aodv_entryPtr_destination_set   ***************/
/////////////////////////////////////////////////////////////////

void 
aodv_entryPtr_destination_set(RoutingTableEntry *entryPtr, int destination)
	{
	/*
	/* Set the destination field of a given entry pointer.
	*/
	FIN(aodv_entryPtr_destination_set(RoutingTableEntry *entryPtr, int destination));
	entryPtr->dest = destination;
	FOUT;
	}
/////////////////////////////////////////////////////////////////
/****************** aodv_entryPtr_nextHop_set  *****************/
/////////////////////////////////////////////////////////////////

void 
aodv_entryPtr_nextHop_set(RoutingTableEntry *entryPtr, int nextHop)
	{
	/*
	/* Set the nextHop field of a given entry pointer.
	*/
	FIN(aodv_entryPtr_nextHop_set(RoutingTableEntry *entryPtr, int nextHop));
	entryPtr->nextHop = nextHop;
	FOUT;
	}
/////////////////////////////////////////////////////////////////
/****************** aodv_entryPtr_status_set  ******************/
/////////////////////////////////////////////////////////////////

void 
aodv_entryPtr_status_set(RoutingTableEntry *entryPtr, int status)
	{
	/*
	/* Set the status field of a given entry pointer.
	*/
	FIN(aodv_entryPtr_status_set(RoutingTableEntry *entryPtr, int status));
	entryPtr->status = status;
	FOUT;
	}

/////////////////////////////////////////////////////////////////
/****************** aodv_entry_status_get  *********************/
/////////////////////////////////////////////////////////////////

int 
aodv_entry_status_get(int dest)
	{
	/*
	/* Return the status of a given entry.
	*/
	RoutingTableEntry *entryPtr;
	FIN(aodv_entry_status_get(int dest));
	 entryPtr= aodv_routingTable_entry_get(dest);
	FRET(entryPtr->status);
	}

////////////////////////////////////////////////////////////////
/******** aodv_entryPtr_expirationInterrupt_cancel ************/
////////////////////////////////////////////////////////////////
	
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
			if(DEBUG > 2 ) printf("      > Event still pending: process cancels it\n");
			op_ev_cancel(entry->expiration_evt);
			if(DEBUG > 2 ) printf("      > Evhandle cancelled\n");
			}
		else
			{
			if(DEBUG > 2 ) printf("      > Event already occured\n");
			}		
		}
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
	void aodv_routing (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Obtype _op_aodv_routing_init (int * init_block_ptr);
	void _op_aodv_routing_diag (OP_SIM_CONTEXT_ARG_OPT);
	void _op_aodv_routing_terminate (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Address _op_aodv_routing_alloc (VosT_Obtype, int);
	void _op_aodv_routing_svar (void *, const char *, void **);


#if defined (__cplusplus)
} /* end of 'extern "C"' */
#endif




/* Process model interrupt handling procedure */


void
aodv_routing (OP_SIM_CONTEXT_ARG_OPT)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	FIN_MT (aodv_routing ());

		{
		/* Temporary Variables */
		int requestedDest, destination, source, downstream_node;
		AckEvt* ackEvt;
		//int intrpt_code;
		int dest = -1, nextHop, previousHop;
		int i,j;
		RoutingTableEntry * entryPtr;
		//Evhandle* evhPtr;
		double first_hello_intrvl;
		Packet* pkptr;
		int type;
		
		Ici* ack_ici;
		int ack_next_hop, ack_final_destination;
		
		//Ici* serve_buffer_ici;
		
		Ici* failure_transmit_ici;
		/* End of Temporary Variables */


		FSM_ENTER ("aodv_routing")

		FSM_BLOCK_SWITCH
			{
			/*---------------------------------------------------------*/
			/** state (Init) enter executives **/
			FSM_STATE_ENTER_FORCED_NOLABEL (0, "Init", "aodv_routing [Init enter execs]")
				FSM_PROFILE_SECTION_IN ("aodv_routing [Init enter execs]", state0_enter_exec)
				{
				/* Initialization of the process model.				 */  
				/* All the attributes are loaded in this routine and */
				/* a self interruption is scheduled to initiate the  */
				/* the first Hello Interval                          */
				
				node_id = op_topo_parent (op_id_self());
				op_ima_obj_attr_get(node_id,"MAC Address",&node_addr);
				op_ima_obj_attr_get(node_id,"DEBUG",                   &DEBUG);
				
				
				op_ima_obj_attr_get(op_id_self(),"ACTIVE_ROUTE_TIMEOUT",&ACTIVE_ROUTE_TIMEOUT);
				op_ima_obj_attr_get(op_id_self(),"HELLO_MODE",          &HELLO_MODE);
				op_ima_obj_attr_get(op_id_self(),"TTL",                 &TTL_START);
				//op_ima_obj_attr_get(op_id_self(),"TR",                  &TR);
				
				net_id = op_topo_parent(node_id);
				
				
				/* Initialize AODV state variables*/
				
				ALLOWED_HELLO_LOSS    = 2;
				HELLO_INTERVAL        = 3;
				LOCAL_ADD_TTL         = 2;
				RREQ_RETRIES          = 2;
				TTL_INCREMENT         = 2;
				TTL_TRESHOLD          = 15;
				MIN_REPAIR_TTL        = 2;
				NET_DIAMETER          = 20;
				NODE_TRAVERSAL_TIME   = 3;//0.6;
				MAX_REPAIR_TTL        = 0.3*NET_DIAMETER;//=6
				NEXT_HOP_WAIT         = NODE_TRAVERSAL_TIME + 0.1;
				NET_TRAVERSAL_TIME    = 3*NODE_TRAVERSAL_TIME*NET_DIAMETER/2; // = 9
				BROADCAST_RECORD_TIME = 2*NET_TRAVERSAL_TIME;
				REV_ROUTE_LIFE        = NET_TRAVERSAL_TIME;
				DELETE_PERIOD         = 5*max_dble(ACTIVE_ROUTE_TIMEOUT, ALLOWED_HELLO_LOSS*HELLO_INTERVAL);
				MY_ROUTE_TIMEOUT      = 2* ACTIVE_ROUTE_TIMEOUT;
				Wait_ACK              = DELETE_PERIOD;
				myBroadcastID         = 0;
				mySeqNb               = 0;
				
				/* Set priorities*/
				op_intrpt_priority_set(OPC_INTRPT_STRM,RCV_STRM,20);
				op_intrpt_priority_set(OPC_INTRPT_STRM,SRC_STRM,5);
				
				/* Initialize state variables */
				
				routingTable= newFifo();
				reverseListOfPrecursors = newFifo();
				//??routeExpirationTimers=newFifo();
				initFifo(&ackEvtFifo);
				
				
				for (i=0;i<N;i++)
					{
					RequestSent[i].sequence_number=0;
					RequestSent[i].status=OFF;
					RequestSent[i].nbOfRetries=0;
					RequestSent[i].ttl=TTL_START;
					RequestSent[i].gratuitous =0;
					
					for(j=0;j<N;j++)
				    	{
						RequestSeen[i][j].expirationTime = -1;
						RequestSeen[i][j].broadcastID = -1;
						}
					}
				
				// Format the Hello Module and schedule the first broadcast of hello msg
				if(HELLO_MODE)
					{
					/*init the hello_module */
					hello_module.hello_msg_template = op_pk_create_fmt("AODV_RREP");
					/* Assign source and dest to the reply packet */
					op_pk_nfd_set(hello_module.hello_msg_template,"SRC",node_addr);
					op_pk_nfd_set(hello_module.hello_msg_template,"DEST",node_addr);
					op_pk_nfd_set(hello_module.hello_msg_template,"HopCount",0);
					op_pk_nfd_set(hello_module.hello_msg_template,"Lifetime",ALLOWED_HELLO_LOSS * HELLO_INTERVAL);
					/* trigger first intrpt for hello broadcast */
					hello_dist = op_dist_load ("uniform_double", 0.0,1.0);
					first_hello_intrvl = op_dist_outcome(hello_dist);
					hello_module.evt= op_intrpt_schedule_self(op_sim_time()+first_hello_intrvl,HELLO_CODE);
					}
				
				/* Initialize stat variables */
				ack = input = output = data_pk_destroyed= data_pk_buffer = 0;
				stats[node_addr].own_input = 0;
				stats[node_addr].forward_input =0;
				stats[node_addr].own_output =0;
				stats[node_addr].forward_output =0;
				stats[node_addr].data_pk_destroyed =0;
				stats[node_addr].data_pk_buffer =0;
				stats[node_addr].ack =0;
				}
				FSM_PROFILE_SECTION_OUT (state0_enter_exec)

			/** state (Init) exit executives **/
			FSM_STATE_EXIT_FORCED (0, "Init", "aodv_routing [Init exit execs]")


			/** state (Init) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "Init", "idle", "tr_0", "aodv_routing [Init -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (idle) enter executives **/
			FSM_STATE_ENTER_UNFORCED (1, "idle", state1_enter_exec, "aodv_routing [idle enter execs]")

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (3,"aodv_routing")


			/** state (idle) exit executives **/
			FSM_STATE_EXIT_UNFORCED (1, "idle", "aodv_routing [idle exit execs]")


			/** state (idle) transition processing **/
			FSM_PROFILE_SECTION_IN ("aodv_routing [idle trans conditions]", state1_trans_conds)
			FSM_INIT_COND (TIMEOUT)
			FSM_TEST_COND (LOWER_LAYER_ARVL)
			FSM_TEST_COND (Route_Expiration)
			FSM_TEST_COND (ACK_ARVL)
			FSM_TEST_COND (NO_REPLY)
			FSM_TEST_COND (UPPER_LAYER_ARVL)
			FSM_TEST_COND (Hello_Intvl_Expiration)
			FSM_TEST_COND (NACK_ARVL)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("idle")
			FSM_PROFILE_SECTION_OUT (state1_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 5, state5_enter_exec, ;, "TIMEOUT", "", "idle", "Ack_Timeout", "tr_27", "aodv_routing [idle -> Ack_Timeout : TIMEOUT / ]")
				FSM_CASE_TRANSIT (1, 2, state2_enter_exec, ;, "LOWER_LAYER_ARVL", "", "idle", "Rcv_Upcoming_Strm", "tr_29", "aodv_routing [idle -> Rcv_Upcoming_Strm : LOWER_LAYER_ARVL / ]")
				FSM_CASE_TRANSIT (2, 6, state6_enter_exec, ;, "Route_Expiration", "", "idle", "Entry_Management", "tr_33", "aodv_routing [idle -> Entry_Management : Route_Expiration / ]")
				FSM_CASE_TRANSIT (3, 4, state4_enter_exec, ;, "ACK_ARVL", "", "idle", "Rcv_Ack", "tr_40", "aodv_routing [idle -> Rcv_Ack : ACK_ARVL / ]")
				FSM_CASE_TRANSIT (4, 3, state3_enter_exec, ;, "NO_REPLY", "", "idle", "Renew_Request", "tr_43", "aodv_routing [idle -> Renew_Request : NO_REPLY / ]")
				FSM_CASE_TRANSIT (5, 7, state7_enter_exec, ;, "UPPER_LAYER_ARVL", "", "idle", "Data_Transmission", "tr_46", "aodv_routing [idle -> Data_Transmission : UPPER_LAYER_ARVL / ]")
				FSM_CASE_TRANSIT (6, 8, state8_enter_exec, ;, "Hello_Intvl_Expiration", "", "idle", "Send_Hello", "tr_54", "aodv_routing [idle -> Send_Hello : Hello_Intvl_Expiration / ]")
				FSM_CASE_TRANSIT (7, 9, state9_enter_exec, ;, "NACK_ARVL", "", "idle", "Failure_Transmit", "tr_59", "aodv_routing [idle -> Failure_Transmit : NACK_ARVL / ]")
				FSM_CASE_TRANSIT (8, 1, state1_enter_exec, ;, "default", "", "idle", "idle", "tr_49", "aodv_routing [idle -> idle : default / ]")
				}
				/*---------------------------------------------------------*/



			/** state (Rcv_Upcoming_Strm) enter executives **/
			FSM_STATE_ENTER_FORCED (2, "Rcv_Upcoming_Strm", state2_enter_exec, "aodv_routing [Rcv_Upcoming_Strm enter execs]")
				FSM_PROFILE_SECTION_IN ("aodv_routing [Rcv_Upcoming_Strm enter execs]", state2_enter_exec)
				{
				/* This state receives the incoming packet stream from   */
				/* the lower layer. It first checks the type of the      */
				/* received packet then, calls the appropriate function  */
				/* to proceed.                                           */
				/* N.B. promiscuous listenning is blocked at this state  */
				
				pkptr = op_pk_get(RCV_STRM);				
				
				op_pk_nfd_get(pkptr,"Type",&type);
				op_pk_nfd_get(pkptr,"NextHop",&nextHop);
				op_pk_nfd_get(pkptr,"PreviousHop",&previousHop);
				
				// check packet type
				switch(type) 
					{				
					case DATA_PACKET_TYPE: // data received 
						{							
						if(nextHop != node_addr)
							{
							if (DEBUG > 3) printf("    - ERROR due to Promiscuous mode : packet not destined for node !\n");
							op_pk_destroy(pkptr);
							}
						else
							{
							if (DEBUG >0) printf("\n{{ node %d }} @ aodv.receive_upcoming_stream ::\n    - A DATA pk has been received from mac layer\n", node_addr);
							if (DEBUG>1) aodv_pk_print(pkptr);
							aodv_data_pk_receive(pkptr);
							}			
						break;
						}
								
					case REQUEST_PACKET_TYPE: // RREQ received
						{
						op_pk_nfd_get(pkptr,"SRC",&source);
						if(source !=node_addr)
							{
							if (DEBUG >0) printf("\n{{ node %d }} @ aodv.receive_upcoming_stream ::\n    - A REQUEST pk has been received from mac layer\n", node_addr);
							if (DEBUG>1) aodv_pk_print(pkptr);
							aodv_rreq_pk_receive(pkptr);
							}
						else
							{		
							// request has looped back to source: RREQ discarded.
							op_pk_destroy(pkptr);
							}
						break;
						}
												
					case REPLY_PACKET_TYPE:  // RREP received
						{
								
						if(nextHop == node_addr)
							{
							// it is a regular RREP 
							if (DEBUG >0) printf("\n{{ node %d }} @ aodv.receive_upcoming_stream ::\n    - A REPLY pk has been received from mac layer\n", node_addr);
							if (DEBUG>1) aodv_pk_print(pkptr);
							aodv_rrep_pk_receive(pkptr);
							}
						else if (nextHop == BROADCAST)
							{
							// it is a Hello Message 
							if (DEBUG >0) printf("\n{{ node %d }} @ aodv.receive_upcoming_stream ::\n    - A HELLO msg has been received from mac layer\n", node_addr);
							if (DEBUG>1) aodv_pk_print(pkptr);
							aodv_hello_msg_receive(pkptr);
							}
						else
							{
							if (DEBUG > 3) printf("    - Promiscuous mode ERROR: packet not destinated to node !\n");
							op_pk_destroy(pkptr);
							}
						break;		
						}
												
					case ERROR_PACKET_TYPE: // RERR received
						{
						if (DEBUG >0) printf("\n{{ node %d }} @ aodv.receive_upcoming_stream ::\n    - An ERROR pk has been received from mac layer\n", node_addr);
						if (DEBUG>1) aodv_pk_print(pkptr);
						aodv_rerr_pk_receive(pkptr); 
						break;
						}						
					} /* end switch(type)*/
				}
				FSM_PROFILE_SECTION_OUT (state2_enter_exec)

			/** state (Rcv_Upcoming_Strm) exit executives **/
			FSM_STATE_EXIT_FORCED (2, "Rcv_Upcoming_Strm", "aodv_routing [Rcv_Upcoming_Strm exit execs]")


			/** state (Rcv_Upcoming_Strm) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "Rcv_Upcoming_Strm", "idle", "tr_32", "aodv_routing [Rcv_Upcoming_Strm -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (Renew_Request) enter executives **/
			FSM_STATE_ENTER_FORCED (3, "Renew_Request", state3_enter_exec, "aodv_routing [Renew_Request enter execs]")
				FSM_PROFILE_SECTION_IN ("aodv_routing [Renew_Request enter execs]", state3_enter_exec)
				{
				/* This state is responsible for requests renewals.       */
				/* When a request is sent, a corresponding entry is       */
				/* created in the RequestSent repository in order to      */
				/* control route requests broadcast. For each request     */
				/* that has been generated, node stores the number of     */
				/* retries, the expiration time (time before renewal),    */
				/* future TTL in case of renewal, the brodcast ID, the    */
				/* sequence number and finally the status of the request. */ 
				/* When the waiting time for a RREP expires, this state   */
				/* checks the status of the request.                      */
				/* 1. If status is REPLIED, there is no need to           */
				/* rebroadcast the request.The state only resets the      */
				/* request attributes within the RequestSent repository.  */
				/* 2. If status is REPAIR, the request has been generated */
				/* following a link beakage. In this case, if no reply    */
				/* was received, node generate a RERR for the             */
				/* corresponding destination.                             */
				/* 3. If status is WAITING, request is still pending but  */
				/* no reply was received within the time frame. Node must */
				/* rebroadcast request and update the RequestSent         */
				/* repository.                                            */
				
				if (DEBUG >0) printf("\n{{ node %d }} @ aodv.renew_request :: \n", node_addr);
				// node didn't receive any reply to its request 	
				
				// read expired request destination
				requestedDest = op_intrpt_code()-NREPLY_CODE;
				
				// check whether entry is under repair or not
				// 1- if status is WAITING_FOR_REPAIR, request is due to a link breakage. 
				// Consequently, "no reply" implies "RERR generation".
				// 2- if status is WAITING_FOR_REPLY. Renew request if poaaible.
				if(RequestSent[requestedDest].status == WAITING_FOR_REPAIR)
					{
					if(DEBUG) printf("    - Error @ node %d: Fail to repair destination %d\n", node_addr,requestedDest);
					// update stats
					data_pk_destroyed+=aodv_buffer_size_get(requestedDest);
					stats[node_addr].data_pk_destroyed+=aodv_buffer_size_get(requestedDest);
					data_pk_buffer -= aodv_buffer_size_get(requestedDest);
					stats[node_addr].data_pk_buffer-=aodv_buffer_size_get(requestedDest);
					// Drop data packets in the buffer
					if (DEBUG) printf("    - Drop data from buffer [%d packets]\n", aodv_buffer_size_get(requestedDest));
					op_subq_flush(requestedDest);
					if (DEBUG) printf("    - Generate RERR for destination %d\n", requestedDest);
						
					// reset requestSent repository for requestedDest			
					RequestSent[requestedDest].nbOfRetries=0;
					RequestSent[requestedDest].ttl=TTL_START;
					RequestSent[requestedDest].status =OFF;		
				
					//generate RERR packet
					aodv_rerr_pk_generate(requestedDest,0);			
				
					}
				else if(RequestSent[requestedDest].status == WAITING_FOR_REPLY) // request WAITING for reply
					{
					RequestSent[requestedDest].status=OFF; 
					if (DEBUG) printf("    - No reply for destination %d [Next TTL = %d ]\n", requestedDest,RequestSent[requestedDest].ttl);
					if(((RequestSent[requestedDest].ttl > TTL_TRESHOLD) && (RequestSent[requestedDest].nbOfRetries <(RREQ_RETRIES+2)))
						|| (RequestSent[requestedDest].ttl <= TTL_TRESHOLD))
						{
						// renew request
						if (DEBUG) printf("    -  Regenerate a RREQ pk for destination %d\n",requestedDest);
						aodv_rreq_pk_regenerate(requestedDest);
						}
					else // data dropped from buffer and "destination unreachable" msg delivered to application
						{
						if (DEBUG) printf("    - Error @ node %d:  No more possible retry for destination %d: dropping buffer [%d packets lost]\n",node_addr,requestedDest,aodv_buffer_size_get(requestedDest));
						//reset the RequestSent repository
						RequestSent[requestedDest].nbOfRetries=0;
						RequestSent[requestedDest].ttl=TTL_START;
						RequestSent[requestedDest].status =OFF;
						// empty buffer
						// before flushing, count nb of packets and increment number of lost packets: to do
						data_pk_destroyed+=aodv_buffer_size_get(requestedDest);
						stats[node_addr].data_pk_destroyed+=aodv_buffer_size_get(requestedDest);
						data_pk_buffer -= aodv_buffer_size_get(requestedDest);
						stats[node_addr].data_pk_buffer-=aodv_buffer_size_get(requestedDest);
						// Drop data packets in the buffer
						if (DEBUG) printf("    - Drop data from buffer [%d packets]\n", aodv_buffer_size_get(requestedDest));
						// flush buffer
						op_subq_flush(requestedDest);		
						}
					}
				}
				FSM_PROFILE_SECTION_OUT (state3_enter_exec)

			/** state (Renew_Request) exit executives **/
			FSM_STATE_EXIT_FORCED (3, "Renew_Request", "aodv_routing [Renew_Request exit execs]")


			/** state (Renew_Request) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "Renew_Request", "idle", "tr_44", "aodv_routing [Renew_Request -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (Rcv_Ack) enter executives **/
			FSM_STATE_ENTER_FORCED (4, "Rcv_Ack", state4_enter_exec, "aodv_routing [Rcv_Ack enter execs]")
				FSM_PROFILE_SECTION_IN ("aodv_routing [Rcv_Ack enter execs]", state4_enter_exec)
				{
				/* Here, we are using a sliding window of 1. So after    */
				/* the node receives an ackowledgement for the previous  */
				/* transmitted packet (destined for a given node), it    */
				/* dequeues the next packet waiting in the buffer and    */
				/* transmitts it. Also, it desactivates the ack-timeout  */ 
				/* timer corresponding to the previous packet trans-     */
				/* mission.                                              */
				
				if (DEBUG >0) printf("\n{{ node %d }} @ aodv.receive_ack ::\n", node_addr);
				// increment number of received acks
				ack++;
				stats[node_addr].ack++; 
				// extract the ICI that is associated with the interruption
				ack_ici = op_intrpt_ici();
				op_ici_attr_get(ack_ici,"Relay_Destination",&ack_next_hop);
				op_ici_attr_get(ack_ici,"Final_Destination",&ack_final_destination);
				op_ici_destroy(ack_ici);
				if (DEBUG>0) printf("    - Ack received from node %d [Final destination is %d]\n",ack_next_hop,ack_final_destination);
				// Excrat associated AckEvt
				if (DEBUG > 3) printf("      - Extracting associated event\n");
				ackEvt=(AckEvt*) getInFifoMultiplex(&ackEvtFifo,ack_final_destination);
				if (ackEvt!=NULL)
					{
					// destroy packet copy
					op_pk_destroy(ackEvt->copy_pk_ptr);
					// cancel event
					if (op_ev_pending(ackEvt->evt) == OPC_TRUE)
						{
						if (op_ev_cancel(ackEvt->evt)==OPC_COMPCODE_SUCCESS)
							if (DEBUG > 3) printf("      - Event has been succesfully cancelled\n");
						}
					else
						if (DEBUG > 3) printf("      - Event already passed away\n");
				 
					op_prg_mem_free(ackEvt);
					}
				else
					if (DEBUG > 3) printf("      - ERROR: event does not exist !\n");
				
				// serve buffer if not empty
				if (aodv_buffer_is_empty(ack_final_destination)==OPC_FALSE)
					{
					if (DEBUG) printf("    - Still %d packets waiting: send next packet in buffer\n", aodv_buffer_size_get(ack_final_destination));
					aodv_buffer_serve(ack_final_destination);
					}
				else
					{
					if (DEBUG) printf("    - No more packets to send: destination %d completely served\n",ack_final_destination);
					}
				}
				FSM_PROFILE_SECTION_OUT (state4_enter_exec)

			/** state (Rcv_Ack) exit executives **/
			FSM_STATE_EXIT_FORCED (4, "Rcv_Ack", "aodv_routing [Rcv_Ack exit execs]")


			/** state (Rcv_Ack) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "Rcv_Ack", "idle", "tr_41", "aodv_routing [Rcv_Ack -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (Ack_Timeout) enter executives **/
			FSM_STATE_ENTER_FORCED (5, "Ack_Timeout", state5_enter_exec, "aodv_routing [Ack_Timeout enter execs]")
				FSM_PROFILE_SECTION_IN ("aodv_routing [Ack_Timeout enter execs]", state5_enter_exec)
				{
				/* A node downstream has not ackowledged a data packet   */
				/* within the appropriate time frame. Current state      */
				/* enqueues a copy of the non-ackowledged data packet    */
				/* in the buffer and attempts to perform local repair on */
				/* the broken link                                       */
				
				if (DEBUG >0) printf("\n{{ node %d }} @ aodv_routing.ack-timeout ::\n", node_addr);
				// Read the downstream node address
				destination = op_intrpt_code();
					
				// Find associated ack event(final destination + duplicata of the lost data packet)
				ackEvt=(AckEvt*) getInFifoMultiplex(&ackEvtFifo,destination);
				if (ackEvt != NULL)
					{
					// read packet final destination
					op_pk_nfd_get(ackEvt->copy_pk_ptr,"DEST", &destination);
					op_pk_nfd_get(ackEvt->copy_pk_ptr,"SRC", &source);
					if(DEBUG >0) printf("    - No ack received for destination %d\n", destination );
					// insert packet into buffer
					if(DEBUG >0) printf("    - Insert copy of the lost packet in buffer\n");
					aodv_data_pk_queue(op_pk_copy(ackEvt->copy_pk_ptr));
					// handle link breakage
					if(DEBUG >0) printf("    - Attempt to repair broken link\n");
					aodv_link_repair_attempt(destination, aodv_entry_hopCount_get(source));
					// delete ack event
					op_prg_mem_free(ackEvt);
					}
				else 
					{
					if(DEBUG) printf("    - No event found for destination %d\n",destination );
					op_sim_end(" no error node found\n","","","");
					}
				
				}
				FSM_PROFILE_SECTION_OUT (state5_enter_exec)

			/** state (Ack_Timeout) exit executives **/
			FSM_STATE_EXIT_FORCED (5, "Ack_Timeout", "aodv_routing [Ack_Timeout exit execs]")


			/** state (Ack_Timeout) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "Ack_Timeout", "idle", "tr_26", "aodv_routing [Ack_Timeout -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (Entry_Management) enter executives **/
			FSM_STATE_ENTER_FORCED (6, "Entry_Management", state6_enter_exec, "aodv_routing [Entry_Management enter execs]")
				FSM_PROFILE_SECTION_IN ("aodv_routing [Entry_Management enter execs]", state6_enter_exec)
				{
				/* This state is in charge of routing table management, */
				/* i.e. invalidation of active entries following an     */
				/* expiration and deletion of invalid entries.          */
				/* N.B. Entries under repair are not deleted.           */
				
				if (DEBUG >0) printf("\n{{ node %d }} @ aodv.entry_management :: \n", node_addr);
				
				// determine the entry that has expired
				dest= op_intrpt_code();
				
				// print entry
				if(DEBUG > 1) aodv_entry_print(dest);
								
				// read entry from routing table 
				entryPtr=aodv_routingTable_entry_get(dest);
						
				if(entryPtr != OPC_NIL) 
					//entry does exist
					{
					if(entryPtr->status == ACTIVE) 
						{				
						if (DEBUG >0) printf("    - Invalidate entry for destination %d (time is %d)\n",entryPtr->dest, op_sim_time());
						// node is not a neighbor: entry invalidated and deletion intrpt is scheduled
						entryPtr->lastHopCount = entryPtr->hopCount;
						entryPtr->destSeqNb++;
						entryPtr->hopCount= INFINITY; 
						entryPtr->status = INVALID;
						if (DEBUG >0) printf("    - Schedule deletion for entry %d\n",entryPtr->dest);
						aodv_entry_expirationTime_set(entryPtr->dest, op_sim_time()+DELETE_PERIOD);			
						}
					else if(entryPtr->status == UNDER_REPAIR)
						{
						entryPtr->lastExpirationTime = entryPtr->expirationTime;
						// entry is under repair: no deletion should be proceeded.
						if (DEBUG >0) printf("    - Entry %d is under repair: do not delete it !\n",entryPtr->dest);
						}
					else // entry has to be deleted
						{
						aodv_routingTable_entry_delete(dest);
						if (DEBUG) printf("    - Entry for destination %d has been deleted\n",dest);		
						}
					}
				else
					if (DEBUG) printf("    - ERROR: entry does not exist !\n");
				// print entry
				if(DEBUG > 1) aodv_entry_print(dest);
				
				}
				FSM_PROFILE_SECTION_OUT (state6_enter_exec)

			/** state (Entry_Management) exit executives **/
			FSM_STATE_EXIT_FORCED (6, "Entry_Management", "aodv_routing [Entry_Management exit execs]")


			/** state (Entry_Management) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "Entry_Management", "idle", "tr_34", "aodv_routing [Entry_Management -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (Data_Transmission) enter executives **/
			FSM_STATE_ENTER_FORCED (7, "Data_Transmission", state7_enter_exec, "aodv_routing [Data_Transmission enter execs]")
				FSM_PROFILE_SECTION_IN ("aodv_routing [Data_Transmission enter execs]", state7_enter_exec)
				{
				/* This state receives incoming packet stream from upper */
				/* layer. It extracts the packet, and calls the function */
				/* aodv_data_pk_route() in order to acheminate the packet*/
				
				pkptr = op_pk_get(SRC_STRM);
				op_pk_nfd_get(pkptr,"DEST",&dest);
				// set buffer size to 30 packet at a time
				if(aodv_buffer_size_get(dest) < 20)
					{
					// data from internal source
					if (DEBUG >0) printf("\n{{ node %d }} @ aodv.data_transmit :: \n", node_addr);
					// node has to route the data packet
					// update stats
					input++;
					stats[node_addr].own_input++;
					// call routing function
					aodv_data_pk_route(pkptr); 
					}
				else
					{
					op_pk_destroy(pkptr);
					}
				}
				FSM_PROFILE_SECTION_OUT (state7_enter_exec)

			/** state (Data_Transmission) exit executives **/
			FSM_STATE_EXIT_FORCED (7, "Data_Transmission", "aodv_routing [Data_Transmission exit execs]")


			/** state (Data_Transmission) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "Data_Transmission", "idle", "tr_47", "aodv_routing [Data_Transmission -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (Send_Hello) enter executives **/
			FSM_STATE_ENTER_FORCED (8, "Send_Hello", state8_enter_exec, "aodv_routing [Send_Hello enter execs]")
				FSM_PROFILE_SECTION_IN ("aodv_routing [Send_Hello enter execs]", state8_enter_exec)
				{
				/* Broadcast a Hello Message and schedule a self */
				/* interrupt to the next hello interval       	 */
				
				if(HELLO_MODE)
					{
					if (DEBUG >0) printf("\n{{ node %d }} @ aodv.send_hello_msg :: \n", node_addr);
					// update the seq number field
					op_pk_nfd_set(hello_module.hello_msg_template,"DestSeqNb",mySeqNb); 
					// send packet
					aodv_pk_send_to_mac_layer(op_pk_copy(hello_module.hello_msg_template),BROADCAST);
					// update hello interval
					hello_module.evt= op_intrpt_schedule_self(op_sim_time()+HELLO_INTERVAL,HELLO_CODE);
					if (DEBUG >3) printf("    - Next hello intrpt scheduled\n");	
					}
				}
				FSM_PROFILE_SECTION_OUT (state8_enter_exec)

			/** state (Send_Hello) exit executives **/
			FSM_STATE_EXIT_FORCED (8, "Send_Hello", "aodv_routing [Send_Hello exit execs]")


			/** state (Send_Hello) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "Send_Hello", "idle", "tr_55", "aodv_routing [Send_Hello -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (Failure_Transmit) enter executives **/
			FSM_STATE_ENTER_FORCED (9, "Failure_Transmit", state9_enter_exec, "aodv_routing [Failure_Transmit enter execs]")
				FSM_PROFILE_SECTION_IN ("aodv_routing [Failure_Transmit enter execs]", state9_enter_exec)
				{
				 /* This state is called when a failure to transmit       */ 
				/* occurs at the MAC Layer. In this case, the MAC        */ 
				/* Layer notifies the upper layer by sending a NACK      */
				/* message containing both final and next hop            */
				/* destinations of the lost data packet. At this         */
				/* point, the following operations are executed:         */
				/* 1. Node invalidates the next hop entry.               */
				/* 2. Node enqueues a copy of the lost packet into       */
				/*    buffer.                                            */
				/* 3. Node cancels the ack-timeout event associated      */
				/*    to the previous data packet (the one that has been */
				/*    lost).                                             */
				/* 4. Node calls the aodv_handle_link_breakage ()        */
				/*    procedure to perform local repair on the broken    */
				/*    link.                                              */
				
				if (DEBUG >0) printf("\n{{ node %d }} @ aodv.failure_to_transmit ::\n", node_addr);
				
				// node received a NACK from MAC layer
				// 1- read associated ICI
				failure_transmit_ici=op_intrpt_ici();
				op_ici_attr_get(failure_transmit_ici,"Relay_Destination",&downstream_node);
				op_ici_attr_get(failure_transmit_ici,"Final_Destination",&destination);
				op_ici_destroy(failure_transmit_ici);
					
				if (DEBUG >0) printf("    - Error @ node %d: Nack received for destination %d [final destination is %d]\n",node_addr,downstream_node, destination);		
				if(DEBUG ) aodv_entry_print(destination);
				
				// Cancel AckEvt associated to lost packet
				if (DEBUG> 3) printf("    - Extract associated Ack Evt\n");
				ackEvt=(AckEvt*) getInFifoMultiplex(&ackEvtFifo,destination);
				if (ackEvt!=NULL)
					{
					// insert packet in buffer
					if (DEBUG) printf("    - Insert a copy of the lost packet into buffer\n");
					op_pk_nfd_get(ackEvt->copy_pk_ptr,"SRC", &source);
					aodv_data_pk_queue(op_pk_copy(ackEvt->copy_pk_ptr));
					// cancel event
					if (op_ev_pending(ackEvt->evt) == OPC_TRUE)
						{
						if (op_ev_cancel(ackEvt->evt)==OPC_COMPCODE_SUCCESS)
							if (DEBUG >3) printf("      - Event has been succesfully cancelled\n");
						}
					else
						if (DEBUG > 3) printf("      - Event already passed away\n");
				 
					op_prg_mem_free(ackEvt);
					}
				else
					if (DEBUG > 3) printf("      - ERROR: event does not exist !\n");
				
				if ((aodv_entry_nextHop_get(destination) != NOT_VALID) && (aodv_entry_nextHop_get(destination) != NON_EXISTENT))
					{
					// Invalidate destination entry
					if (DEBUG) printf("    - Invalidate entry\n");
					aodv_entry_invalidate(destination, aodv_entry_destSeqNb_get(destination)+1, 1);
				
					// Handle link breakage
					if (DEBUG) printf("    - Attempt to repair the broken link [destination %d]\n", destination);
					aodv_link_repair_attempt(destination, aodv_entry_hopCount_get(source));
					}
				}
				FSM_PROFILE_SECTION_OUT (state9_enter_exec)

			/** state (Failure_Transmit) exit executives **/
			FSM_STATE_EXIT_FORCED (9, "Failure_Transmit", "aodv_routing [Failure_Transmit exit execs]")


			/** state (Failure_Transmit) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "Failure_Transmit", "idle", "tr_60", "aodv_routing [Failure_Transmit -> idle : default / ]")
				/*---------------------------------------------------------*/



			}


		FSM_EXIT (0,"aodv_routing")
		}
	}




void
_op_aodv_routing_diag (OP_SIM_CONTEXT_ARG_OPT)
	{
	/* No Diagnostic Block */
	}




void
_op_aodv_routing_terminate (OP_SIM_CONTEXT_ARG_OPT)
	{

	FIN_MT (_op_aodv_routing_terminate ())


	/* No Termination Block */

	Vos_Poolmem_Dealloc (op_sv_ptr);

	FOUT
	}


/* Undefine shortcuts to state variables to avoid */
/* syntax error in direct access to fields of */
/* local variable prs_ptr in _op_aodv_routing_svar function. */
#undef node_id
#undef node_addr
#undef net_id
#undef MY_ROUTE_TIMEOUT
#undef ACTIVE_ROUTE_TIMEOUT
#undef ALLOWED_HELLO_LOSS
#undef BROADCAST_RECORD_TIME
#undef HELLO_INTERVAL
#undef LOCAL_ADD_TTL
#undef MAX_REPAIR_TTL
#undef MIN_REPAIR_TTL
#undef NET_DIAMETER
#undef NEXT_HOP_WAIT
#undef NODE_TRAVERSAL_TIME
#undef NET_TRAVERSAL_TIME
#undef REV_ROUTE_LIFE
#undef RREQ_RETRIES
#undef TTL_INCREMENT
#undef TTL_TRESHOLD
#undef TTL_START
#undef DELETE_PERIOD
#undef TR
#undef DEBUG
#undef myBroadcastID
#undef mySeqNb
#undef RequestSent
#undef RequestSeen
#undef Wait_ACK
#undef routingTable
#undef reverseListOfPrecursors
#undef ackEvtFifo
#undef HELLO_MODE
#undef hello_module
#undef hello_dist

#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE

#define FIN_PREAMBLE_DEC
#define FIN_PREAMBLE_CODE

VosT_Obtype
_op_aodv_routing_init (int * init_block_ptr)
	{
	VosT_Obtype obtype = OPC_NIL;
	FIN_MT (_op_aodv_routing_init (init_block_ptr))

	obtype = Vos_Define_Object_Prstate ("proc state vars (aodv_routing)",
		sizeof (aodv_routing_state));
	*init_block_ptr = 0;

	FRET (obtype)
	}

VosT_Address
_op_aodv_routing_alloc (VosT_Obtype obtype, int init_block)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	aodv_routing_state * ptr;
	FIN_MT (_op_aodv_routing_alloc (obtype))

	ptr = (aodv_routing_state *)Vos_Alloc_Object (obtype);
	if (ptr != OPC_NIL)
		{
		ptr->_op_current_block = init_block;
#if defined (OPD_ALLOW_ODB)
		ptr->_op_current_state = "aodv_routing [Init enter execs]";
#endif
		}
	FRET ((VosT_Address)ptr)
	}



void
_op_aodv_routing_svar (void * gen_ptr, const char * var_name, void ** var_p_ptr)
	{
	aodv_routing_state		*prs_ptr;

	FIN_MT (_op_aodv_routing_svar (gen_ptr, var_name, var_p_ptr))

	if (var_name == OPC_NIL)
		{
		*var_p_ptr = (void *)OPC_NIL;
		FOUT
		}
	prs_ptr = (aodv_routing_state *)gen_ptr;

	if (strcmp ("node_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->node_id);
		FOUT
		}
	if (strcmp ("node_addr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->node_addr);
		FOUT
		}
	if (strcmp ("net_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->net_id);
		FOUT
		}
	if (strcmp ("MY_ROUTE_TIMEOUT" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->MY_ROUTE_TIMEOUT);
		FOUT
		}
	if (strcmp ("ACTIVE_ROUTE_TIMEOUT" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ACTIVE_ROUTE_TIMEOUT);
		FOUT
		}
	if (strcmp ("ALLOWED_HELLO_LOSS" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ALLOWED_HELLO_LOSS);
		FOUT
		}
	if (strcmp ("BROADCAST_RECORD_TIME" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->BROADCAST_RECORD_TIME);
		FOUT
		}
	if (strcmp ("HELLO_INTERVAL" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->HELLO_INTERVAL);
		FOUT
		}
	if (strcmp ("LOCAL_ADD_TTL" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->LOCAL_ADD_TTL);
		FOUT
		}
	if (strcmp ("MAX_REPAIR_TTL" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->MAX_REPAIR_TTL);
		FOUT
		}
	if (strcmp ("MIN_REPAIR_TTL" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->MIN_REPAIR_TTL);
		FOUT
		}
	if (strcmp ("NET_DIAMETER" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->NET_DIAMETER);
		FOUT
		}
	if (strcmp ("NEXT_HOP_WAIT" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->NEXT_HOP_WAIT);
		FOUT
		}
	if (strcmp ("NODE_TRAVERSAL_TIME" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->NODE_TRAVERSAL_TIME);
		FOUT
		}
	if (strcmp ("NET_TRAVERSAL_TIME" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->NET_TRAVERSAL_TIME);
		FOUT
		}
	if (strcmp ("REV_ROUTE_LIFE" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->REV_ROUTE_LIFE);
		FOUT
		}
	if (strcmp ("RREQ_RETRIES" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->RREQ_RETRIES);
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
	if (strcmp ("TTL_START" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->TTL_START);
		FOUT
		}
	if (strcmp ("DELETE_PERIOD" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->DELETE_PERIOD);
		FOUT
		}
	if (strcmp ("TR" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->TR);
		FOUT
		}
	if (strcmp ("DEBUG" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->DEBUG);
		FOUT
		}
	if (strcmp ("myBroadcastID" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->myBroadcastID);
		FOUT
		}
	if (strcmp ("mySeqNb" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->mySeqNb);
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
	if (strcmp ("Wait_ACK" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->Wait_ACK);
		FOUT
		}
	if (strcmp ("routingTable" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->routingTable);
		FOUT
		}
	if (strcmp ("reverseListOfPrecursors" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->reverseListOfPrecursors);
		FOUT
		}
	if (strcmp ("ackEvtFifo" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ackEvtFifo);
		FOUT
		}
	if (strcmp ("HELLO_MODE" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->HELLO_MODE);
		FOUT
		}
	if (strcmp ("hello_module" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->hello_module);
		FOUT
		}
	if (strcmp ("hello_dist" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->hello_dist);
		FOUT
		}
	*var_p_ptr = (void *)OPC_NIL;

	FOUT
	}

