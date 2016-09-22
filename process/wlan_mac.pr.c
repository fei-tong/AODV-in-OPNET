/* Process model C form file: wlan_mac.pr.c */
/* Portions of this file copyright 1986-2008 by OPNET Technologies, Inc. */



/* This variable carries the header into the object file */
const char wlan_mac_pr_c [] = "MIL_3_Tfile_Hdr_ 145A 30A modeler 7 4CD10079 4CD10079 1 china-0f9728557 Administrator 0 0 none none 0 0 none 0 0 0 0 0 0 0 0 1e80 8                                                                                                                                                                                                                                                                                                                                                                                            ";
#include <string.h>



/* OPNET system definitions */
#include <opnet.h>



/* Header Block */

/** Include files **/

#include <math.h>
#include <string.h>
#include "oms_pr.h"
#include "oms_tan.h"
#include "oms_bgutil.h"
#include "wlan_support.h"
#include "oms_auto_addr_support.h"
#include "oms_dist_support.h"
#include "oms_dim_stat_support.h"
#include "bridge_header.h"
#include "jammers.h"	
#include "prg_mapping.h"
#include <prg_geo.h>
#include "apptrack.h" 

//自添加
	//$$$$$$$$$$$$$$$$$$ DSR $$$$$$$$$$$$$$$$$$$$$$$$
// define packet Types
#define DATA_PACKET_TYPE 5
#define REQUEST_PACKET_TYPE 7
#define REPLY_PACKET_TYPE 11
#define ERROR_PACKET_TYPE 13

// remote intrpt or self intrpt codes definition
#define ACK_CODE 10000
#define ERROR_CODE 20000
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//自添加结束

/** Constants **/

/* Incoming statistics and stream wires.							*/
#define 	TRANSMITTER_BUSY_INSTAT		1
#define		LOW_LAYER_INPUT_STREAM		0
#define		LOW_LAYER_OUTPUT_STREAM		0

/* Special value indicating that the number of back-off slots are	*/
/* not determined yet.												*/
#define		BACKOFF_SLOTS_UNSET			-1.0

/* Special value for nav_reset_time when NAV is not set based on a	*/
/* received RTS.													*/
#define		NAV_RESET_TIME_UNSET		-1.0
	
/* Define a small value (= 1 psec), which will be used to recover	*/
/* from double arithmetic precision losts while doing time related	*/
/* precision sensitive computations.								*/
#define		PRECISION_RECOVERY			0.000000000001

/** Enumerated Types **/

/* Define interrupt codes for generating handling interrupts					*/
/* indicating changes in deference, frame timeout which infers         			*/
/* that the collision has occurred, random backoff and transmission 			*/
/* completion by the physical layer (self interrupts).							*/
typedef enum WlanT_Mac_Intrpt_Code
	{	
	WlanC_Deference_Off,  	/* Deference before frame transmission 				*/
	WlanC_Frame_Timeout,	/* No frame rcvd in set duration (infer collision)	*/
	WlanC_Backoff_Elapsed,  /* Backoff done before frame transmission			*/
	WlanC_CW_Elapsed,		/* Backoff done after successful frame transmission	*/	
	WlanC_Beacon_Tx_Time,	/* Time to transmit beacon frame                    */
	WlanC_Scan_Timeout,		/* End of scan duration for given channel 			*/
	WlanC_AP_Check_Timeout,	/* Time to check the connectivity status with the	*/
							/* current AP.										*/
	WlanC_NAV_Reset_Time,	/* Time to reset NAV that is updated by an RTS.		*/
	WlanC_Cfp_End			/* End of the Contention free period 				*/
	} WlanT_Mac_Intrpt_Code;


/** Data Structures **/

/* This structure contains all the flags used in this process model to determine	*/
/* various conditions as mentioned in the comments for each flag					*/
typedef struct WlanT_Mac_Flags
	{
	Boolean 	data_frame_to_send; /* Flag to check when station needs to transmit.		*/ 
	Boolean     backoff_flag;  	    /* Backoff flag is set when either the collision is		*/
	                                /* inferred or the channel switched from busy to idle	*/
	Boolean		frame_size_req_rts;	/* Flag that is set when the current frame in process	*/
									/* of transmission is larger than the RTS threshold.	*/
	Boolean		rts_sent;   		/* Flag to indicate that whether the RTS for this		*/
								    /* particular data frame is sent and CTS is received.	*/
	Boolean		rcvd_bad_packet;	/* Flag to indicate that the received packet is bad		*/
	Boolean		bad_packet_dropped;	/* Set this flag if the received packet is marked as 	*/
									/* bad and dropped by this MAC. This flag is used only 	*/
									/* when we are transmitting a frame.					*/
    Boolean	    receiver_busy;		/* Set this flag if receiver busy stat is enabled		*/	
    Boolean	    transmitter_busy;	/* Set this flag if we are transmitting something.		*/	
	Boolean		wait_eifs_dur;		/* Set this flag if the station needs to wait for eifs	*/
									/* duration.											*/	
	Boolean		gateway_flag;		/* Set this flag if the station is a gateway.			*/
	Boolean		bridge_flag;		/* Set this flag if the station is a bridge				*/
	Boolean		immediate_xmt;		/* Set this flag if the new frame can be transmitted	*/
									/* without deferring.									*/
	Boolean		forced_bk_end;		/* Special case: resume with completion of back-off (or	*/
									/* CW) period regardless of receiver's status.			*/
	Boolean		cw_required;		/* Indicates that the MAC is in contention window		*/
									/* period following a successful transmission.			*/
	Boolean		perform_cw;			/* Flag that triggers backoff process for CW period.	*/
	Boolean		nav_updated;		/* Indicates a new NAV value since the last time when	*/
									/* self interrupt is scheduled for the end of deference.*/
	Boolean		collision;			/* Set this flag if a channel became busy while another	*/
									/* one busy.											*/
	Boolean		duration_zero;		/* Set this flag if duration should be zero in next ack	*/
	Boolean		ignore_busy;		/* Set this flag if the STA should ignore receiver busy	*/
	Boolean		tx_beacon;          /* Set this flag if time to send a beacon               */
	Boolean		tx_cf_end;          /* Set this flag if time to send a CF End frame         */
	Boolean		pcf_active;         /* Set this flag for AP if PCF is currently in effect	*/
	Boolean		polled;		        /* Set this flag if the station has received a poll     */
	Boolean		more_data;			/* Set this flag if must poll for more data (MSDU)		*/
	Boolean		more_frag;			/* Set this flag if must poll for more fragments		*/
	Boolean		pcf_side_traf;		/* Set this flag if the AP detects STA to STA traffic   */
	Boolean		active_poll;		/* Set this flag if an active poll is outstanding		*/
	Boolean		scanning;			/* Set this flag while running the scanning procedure	*/
									/* to look for a new access point connectivity.			*/
	Boolean		non_erp_present;	/* Set this flag if we are an ERP STA and our BSS has	*/
									/* at least one non-ERP STA.							*/
	Boolean		cts_to_self;		/* Set this flag if the optional 11g CTS-to-self		*/
									/* protection mechanism is enabled for this MAC.		*/
	Boolean		wait_signal_ext;	/* Set if current tx requires 11g signal extension.		*/
	Boolean		rcvd_bad_cts;		/* Set if a reception is detected during the last		*/
									/* CTS-to-self transmission.							*/
	Boolean		pcf_lowered_drate;	/* Set by an 11g AP, if the data rate of current tx is	*/
									/* lowered due to a piggybacked ACK for a non-ERP STA.	*/
	} WlanT_Mac_Flags;

/* This structure contains the destination address to which the received	*/
/* data packet needs to be sent and the contents of the received packet		*/
/* from the higher layer.												 	*/
typedef struct WlanT_Hld_List_Elem
	{
	double		time_rcvd;  			/* Time packet is received by the higher layer.		*/
	OpT_Int64	destination_address; 	/* Station to which this packet needs to be sent.	*/
	OpT_Int64	origination_address;	/* Packet's original source address (used by APs).	*/
	int			protocol_type;			/* Protocol information of higher layer data.		*/
	Packet*     pkptr;				 	/* Store packet contents.							*/
	} WlanT_Hld_List_Elem;

/* Define a structure to maintain data fragments received by each station	*/
/* for the purpose of reassembly (or defragmentation).		  				*/
typedef struct WlanT_Mac_Defragmentation_Buffer_Entry
	{		
	OpT_Int64	tx_station_address;		/* Store the address of transmitting station.		*/	 
	double		time_rcvd;				/* Store time the last frag for this frame was rcvd.*/ 
	Sbhandle	reassembly_buffer_ptr;	/* Store data fragments for a particular packet.	*/  		 
	} WlanT_Mac_Defragmentation_Buffer_Entry;

/* Information record kept by APs and STAs in IBSSs for each other STA in	*/
/* their BSSs in order to detect the frames that are received as duplicates	*/
/* and to check their capabilities when needed.								*/
typedef struct WlanT_Peer_Info
	{
	Boolean		is_erp;
	int			seq_cntl;
	} WlanT_Peer_Info;

/** Macro Definitions **/

/* Macro definitions to compute the PLCP overhead for control and data frames.	*/
#define	PLCP_OVERHEAD_CTRL(size)				(phy_type == WlanC_11b_PHY ? plcp_overhead_control : wlan_non_11b_plcp_overhead_compute (size, control_data_rate))
#define	PLCP_OVERHEAD_DATA(size)				(phy_type == WlanC_11b_PHY ? plcp_overhead_data    : wlan_non_11b_plcp_overhead_compute (size, operational_speed))
#define	PLCP_OVERHEAD_CTRL_DR(size,data_rate)	(phy_type == WlanC_11b_PHY ? plcp_overhead_control : wlan_non_11b_plcp_overhead_compute (size, data_rate))

/* Macro definitions to compute the transmission delay of control and data		*/
/* frames.																		*/
#define	TXTIME_CTRL(size)				(PLCP_OVERHEAD_CTRL (size) + (double) size / control_data_rate + \
										 ((phy_char_flag == WlanC_ERP_OFDM_11g && control_data_rate > 5500000.0 && control_data_rate != 11000000.0) ? \
										  WLANC_11g_SIGNAL_EXTENSION : 0.0))						
#define	TXTIME_DATA(size)				(PLCP_OVERHEAD_DATA (size) + (double) size / operational_speed + \
										 ((phy_char_flag == WlanC_ERP_OFDM_11g && operational_speed > 5500000.0 && operational_speed != 11000000.0) ? \
										  WLANC_11g_SIGNAL_EXTENSION : 0.0))
#define	TXTIME_CTRL_DR(size,data_rate)	(PLCP_OVERHEAD_CTRL_DR (size, data_rate) + (double) size / data_rate + \
										 ((phy_char_flag == WlanC_ERP_OFDM_11g && data_rate > 5500000.0 && data_rate != 11000000.0) ? \
										  WLANC_11g_SIGNAL_EXTENSION : 0.0))

/* The size of the step between two non-overlapping WLAN channels in terms of	*/
/* channel numbers.																*/
#define WLANC_CH_STEP_FOR_NO_OVERLAP	((int) ceil (channel_bandwidth / channel_spacing))	

/* Macro to check whether WLAN beacon transmission efficiency mode is on.		*/						
#define	BEACON_TX_EFFICIENCY_ENABLED	(beacon_tx_count != WLANC_PERIODIC_BEACON_TX)
						
						
/**	State Transitions **/

/** The data frame send flag is set whenever there is a data to be send by	**/
/** the higher layer or the response frame needs to be sent. However, in 	**/
/** either case the flag will not be set if the receiver is busy			**/
/** Frames cannot be transmitted until medium is idle. Once, the medium 	**/
/** is available then the station is eligible to transmit provided there	**/
/** is a need for backoff. Once the transmission is complete then the		**/
/** station will wait for the response provided the frame transmitted  		**/
/** requires a response (such as RTS and Data frames). If response			**/
/** is not needed then the station will defer to transmit next packet		**/

/* After receiving a stream interrupt, we need to switch states from	*/
/* idle to defer or transmit if there is a frame to transmit and the	*/
/* receiver is not busy													*/ 
/* If a frame is received indicating that the STA should scan, all bets */
/* are off, and the STA moves into the scan state to look for other APs */
#define READY_TO_TRANSMIT		(((intrpt_type == OPC_INTRPT_STRM && wlan_flags->data_frame_to_send == OPC_TRUE && \
								   (pcf_flag == OPC_BOOLINT_DISABLED || (wlan_flags->pcf_active == OPC_FALSE && \
								    (ap_flag == OPC_BOOLINT_ENABLED || cfp_ap_medium_control == OPC_FALSE)))) || \
								  fresp_to_send != WlanC_None || \
								  wlan_flags->polled == OPC_TRUE || \
								  wlan_flags->tx_beacon == OPC_TRUE || \
								  (wlan_flags->pcf_active == OPC_TRUE && ap_flag == OPC_BOOLINT_ENABLED)) && \
								 AP_CONNECTED)

/* When we have a frame to transmit, we move to transmit state if the	*/
/* medium was idle for at least a DIFS time, otherwise we go to defer	*/
/* state.																*/
#define MEDIUM_IS_IDLE			(wlan_flags->receiver_busy  == OPC_FALSE                          && \
								 (current_time - rcv_idle_time + PRECISION_RECOVERY >= difs_time) && \
								 (current_time - nav_duration  + PRECISION_RECOVERY >= difs_time) && \
								 cfp_ap_medium_control      == OPC_FALSE						  && \
								 wlan_flags->pcf_active     == OPC_FALSE						  && \
								 (wlan_flags->wait_eifs_dur == OPC_FALSE || current_time - rcv_idle_time + PRECISION_RECOVERY >= eifs_time))

/* Change state to Defer from Frm_End, if the input buffers are not empty or a frame needs	*/
/* to be retransmitted or the station has to respond to some frame.							*/		
#define FRAME_TO_TRANSMIT		(wlan_flags->data_frame_to_send == OPC_TRUE || fresp_to_send != WlanC_None || \
								 short_retry_count + long_retry_count > 0 || wlan_flags->tx_beacon == OPC_TRUE || \
								 wlan_flags->cw_required == OPC_TRUE)
	
/* After deferring for either collision avoidance or interframe gap		*/
/* the channel will be available for transmission. 						*/
#define DEFERENCE_OFF			(intrpt_type == OPC_INTRPT_SELF                                              && \
								 (intrpt_code == WlanC_Deference_Off || intrpt_code == WlanC_NAV_Reset_Time) && \
								 (wlan_flags->receiver_busy == OPC_FALSE || fresp_to_send == WlanC_Ack))

/* Issue a transmission complete stat once the packet has successfully 	*/
/* been transmitted from the source station								*/						 
#define TRANSMISSION_COMPLETE	(intrpt_type == OPC_INTRPT_STAT && \
								 op_intrpt_stat () == TRANSMITTER_BUSY_INSTAT)

/* Backoff is performed based on the value of the backoff flag.			*/
#define PERFORM_BACKOFF			(wlan_flags->backoff_flag == OPC_TRUE || wlan_flags->perform_cw == OPC_TRUE)

/* Need to start transmitting frame once the backoff (self intrpt) 		*/
/* completed															*/
#define BACKOFF_COMPLETED		(intrpt_type == OPC_INTRPT_SELF && intrpt_code == WlanC_Backoff_Elapsed && \
								 (wlan_flags->receiver_busy == OPC_FALSE || wlan_flags->forced_bk_end == OPC_TRUE))

/* Contention Window period, which follows a successful packet			*/
/* transmission, is completed.											*/
#define CW_COMPLETED			(intrpt_type == OPC_INTRPT_SELF && intrpt_code == WlanC_CW_Elapsed && \
								 (wlan_flags->receiver_busy == OPC_FALSE || wlan_flags->forced_bk_end == OPC_TRUE))

/* After transmission the station will wait for a frame response for    */
/* Data and Rts frames.												    */
#define WAIT_FOR_FRAME          (expected_frame_type != WlanC_None)

/* Need to retransmit frame if there is a frame timeout and the         */
/* required frame is not (successfully) received						*/
#define FRAME_TIMEOUT           ((intrpt_type == OPC_INTRPT_SELF && intrpt_code == WlanC_Frame_Timeout) || \
								 bad_cts_to_self_rcvd == OPC_TRUE)

/* If the frame is received appropriate response will be transmitted    */
/* provided the medium is considered to be idle						    */
#define FRAME_RCVD			    (intrpt_type == OPC_INTRPT_STRM && bad_packet_rcvd == OPC_FALSE && \
		 						 i_strm == LOW_LAYER_INPUT_STREAM)

/* Skip backoff if no backoff is needed								    */
#define TRANSMIT_FRAME			(!PERFORM_BACKOFF)

/* Macros that check the status of AP connectivity.						*/
#define AP_CONNECTED 			(roam_state_ptr->scan_mode == OPC_FALSE)

#define AP_DISCONNECTED 		(roam_state_ptr->scan_mode == OPC_TRUE)

/* Macro that check whether we have a higher layer frame to send.		*/ 
#define DATA_FRAME_TO_TX 		(wlan_flags->data_frame_to_send == OPC_TRUE)

/* When the contention window period is over then we go to IDLE state	*/
/* if we don't have another frame to send at that moment. If we have	*/
/* one then we go to TRANSMIT state if we did not sense any activity	*/
/* on our receiver for a period that is greater than or equal to DIFS	*/
/* period; otherwise we go to DEFER state to defer and back-off before	*/
/* transmitting the new frame.											*/
#define	BACK_TO_IDLE			(CW_COMPLETED && wlan_flags->data_frame_to_send == OPC_FALSE && AP_CONNECTED)
	
#define SEND_NEW_FRAME_AFTER_CW	(CW_COMPLETED && wlan_flags->data_frame_to_send == OPC_TRUE)

/* If we completed a contention window backoff without any more data	*/
/* frames to send, and if AP is disconnected, then start the scanning	*/
/* procedure.															*/
#define SCAN_AFTER_CW			(CW_COMPLETED && wlan_flags->data_frame_to_send == OPC_FALSE && AP_DISCONNECTED)

/* Macros that check the change in the busy status of the receiver.	   	*/
#define	RECEIVER_BUSY_HIGH		(intrpt_type == OPC_INTRPT_STAT && intrpt_code < TRANSMITTER_BUSY_INSTAT && \
								 op_stat_local_read (intrpt_code) > rx_power_threshold && !wlan_flags->collision)

#define	RECEIVER_BUSY_LOW		(((intrpt_type == OPC_INTRPT_STAT && intrpt_code < TRANSMITTER_BUSY_INSTAT) || 			\
								  (intrpt_type == OPC_INTRPT_STRM && i_strm != instrm_from_mac_if)			   ) && 	\
	                             !wlan_flags->receiver_busy)

/* When the backoff procedure is complete start the transmission.		*/
#define	PERFORM_TRANSMIT		((BACKOFF_COMPLETED || SEND_NEW_FRAME_AFTER_CW))

/* Move back to DEFER state if the backoff procedure is interrupted		*/
/* before completion.													*/
#define	BACK_TO_DEFER			(FRAME_RCVD || (wlan_flags->tx_beacon == OPC_TRUE && !wlan_flags->receiver_busy))

/* Macro to evaluate whether the MAC is in a contention free period.	*/
#define	IN_CFP					(pcf_flag == OPC_BOOLINT_ENABLED && \
								 (cfp_ap_medium_control == OPC_TRUE || wlan_flags->pcf_active == OPC_TRUE))

/* After receiving a packet that indicates the end of the current CFP	*/
/* go to back to IDLE state if there is no packet to transmit in the CP.*/
#define	IDLE_AFTER_CFP			(intrpt_type == OPC_INTRPT_STRM && !FRAME_TO_TRANSMIT && !IN_CFP)

/* Macro to cancel the self interrupt for end of deference. It is		*/
/* called at the state transition from DEFER to IDLE.					*/
#define	CANCEL_DEF_EVENT		(op_ev_cancel (deference_evh))

/* If we are not waiting for a response frame and if AP is				*/
/* disconnected, start the scanning procedure.							*/
#define FRM_END_TO_SCAN			(expected_frame_type == WlanC_None && AP_DISCONNECTED)

/* Go to IDLE state if we have no more frames to transmit.				*/
#define FRM_END_TO_IDLE			(!FRAME_TO_TRANSMIT && expected_frame_type == WlanC_None && !IN_CFP && AP_CONNECTED)

/* Move the DEFER state to defer for the next transmission or			*/
/* retransmission.														*/
#define	FRM_END_TO_DEFER		(expected_frame_type == WlanC_None && (FRAME_TO_TRANSMIT || IN_CFP) && AP_CONNECTED)


/** Function Prototypes **/
static void			wlan_mac_sv_init (void);
static void			wlan_transceiver_channel_init (void);
static void			wlan_higher_layer_data_arrival (void);
static void			wlan_hl_packet_drop (Packet* hld_pkptr, OpT_Packet_Size data_size);
static void			wlan_hlpk_enqueue (Packet* hld_pkptr, OpT_Int64 dest_addr, OpT_Int64 orig_src_addr, int protocol_type, OpT_Packet_Size data_size, Boolean polling);
static void			wlan_frame_transmit (void);
static double		wlan_non_11b_plcp_overhead_compute (OpT_Packet_Size mpdu_length, double data_rate);
static double		wlan_plcp_overhead_ofdm_compute (OpT_Packet_Size mpdu_length, double data_rate);
static Boolean		wlan_dest_is_11g_enabled (OpT_Int64 dest_mac_addr);
static void 		wlan_prepare_frame_to_send (WlanT_Mac_Frame_Type frame_type);
static void			wlan_slot_time_set (double new_slot_time);
static void			wlan_interrupts_process (void);
static void			wlan_physical_layer_data_arrival (void);
static Boolean		wlan_tuple_find (OpT_Int64 sta_addr, int seq_cntl, OpT_Int64 dest_addr, Boolean retry);
static void			wlan_data_process (Packet* seg_pkptr, OpT_Int64 dest_addr, OpT_Int64 sta_addr, OpT_Int64 final_dest_addr, int protocol_type, int frag_num, int more_frag, OpT_Packet_Id pkt_id);
static void			wlan_accepted_frame_stats_update (Packet* seg_pkptr);
static void			wlan_schedule_deference (void);
static void			wlan_frame_discard (void);
static void			wlan_pcf_frame_discard (void);
static void			wlan_mac_rcv_channel_status_update (int channel_id);
static Boolean		wlan_poll_list_member_find (OpT_Int64 dest_addr); 

static void			wlan_sta_addr_register (int bss_idx, OpT_Int64 sta_addr, int sta_is_ap, Objid sta_mac_objid, WlanT_Phy_Char_Code phy_char);
static void			wlan_begin_new_scan (void);
static void			wlan_ap_switch (void);
static void 		wlan_sta_addr_deregister (int bss_idx, OpT_Int64 sta_addr);
static void			wlan_reset_sv (void);
static void 		wlan_find_new_ap_virtual (void);

/* Callback functions		*/
#if defined (__cplusplus)
extern "C" {
#endif

static int	   		wlan_hld_list_elem_add_comp (const void* list_elem_ptr1, const void* list_elem_ptr2);

#if defined (__cplusplus)
} /* end of 'extern "C" {' */
#endif

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
	int	                    		intrpt_type                                     ;	/* Intrpt type is stored in this variable */
	WlanT_Mac_Intrpt_Code	  		intrpt_code                                     ;	/* Enumerated intrpt code for interrupts */
	OpT_Int64	              		my_address                                      ;	/* Station's own address */
	Objid	                  		my_objid                                        ;	/* The object ID of the surrounding module. */
	Objid	                  		my_node_objid                                   ;	/* The object ID of the surrounding node. */
	Objid	                  		my_subnet_objid                                 ;	/* The object ID of the subnet in which the */
	                        		                                                	/* surrounding node model resides.          */
	Objid	                  		tx_objid                                        ;	/* Object ID of the radio transmitter that */
	                        		                                                	/* we are connected to.                    */
	Objid	                  		txch_objid                                      ;	/* Object ID of the channel of our transmitter. */
	Objid	                  		rx_objid                                        ;	/* Object ID of the radio receiver that we */
	                        		                                                	/* are connected to.                       */
	Objid	                  		rxch_objid                                      ;	/* Object ID of the channel of our receiver. */
	OmsT_Pr_Handle	         		own_process_record_handle                       ;	/* Handle to the own process record in the network wide process */
	                        		                                                	/* registery.                                                   */
	List*	                  		hld_list_ptr                                    ;	/* Higher layer data arrival queue or list */
	double	                 		data_tx_rate                                    ;	/* Used in storing data rate attribute. This is the rate at which */
	                        		                                                	/* data frame is transmitted.                                     */
	double	                 		operational_speed                               ;	/* Current data rate used for the transmissions of the data frames. */
	                        		                                                	/* This rate can be different than the value of data_tx_rate if the */
	                        		                                                	/* current connected access point can't support that rate.          */
	double	                 		control_data_rate                               ;	/* Data rate used for the transmission of control frames. The rate */
	                        		                                                	/* is the lowest one of the mandatory data rates set of the        */
	                        		                                                	/* configured physical layer technology.                           */
	double	                 		rcvd_frame_drate                                ;	/* Data rate of the last successfully received frame, which is used */
	                        		                                                	/* in determining the transmission rate of the control response     */
	                        		                                                	/* messages (CTS and ACK).                                          */
	int	                    		frag_threshold                                  ;	/* Used in storing fragmentation threshold attribute */
	int	                    		packet_seq_counter                              ;	/* Counter to determine packet sequence number for each */
	                        		                                                	/* transmitted packet.                                  */
	int	                    		packet_seq_control                              ;	/* Sequence control number of the currently transmitted */
	                        		                                                	/* packet.                                              */
	OpT_Int64	              		dcf_destination_addr                            ;	/* Used in storing destination address for DCF transmissions. */
	OpT_Int64	              		dcf_orig_source_addr                            ;	/* Stores the address of the source STA for packets relayed by AP. */
	int	                    		hl_protocol_dcf                                 ;	/* Protocol information of higher layer data currenly processed by DCF. */
	OpT_Int64	              		pcf_destination_addr                            ;	/* Used in storing destination address for CFP transmissions. */
	OpT_Int64	              		pcf_orig_source_addr                            ;	/* Stores the address of the source STA for packets relayed by AP */
	                        		                                                	/* during CFPs.                                                   */
	int	                    		hl_protocol_pcf                                 ;	/* Protocol information of higher layer data currenly processed by PCF. */
	int	                    		packet_frag_number                              ;	/* Counter to determine fragment number for each DCF packet fragment. */
	int	                    		pcf_packet_frag_number                          ;	/* Counter to determine fragment number for each CFP packet fragment. */
	Sbhandle	               		fragmentation_buffer_ptr                        ;	/* Fragmentation buffer used to store transmit packet fragments. */
	Sbhandle	               		common_rsmbuf_ptr                               ;	/* Common reassembly buffer used for segments containing the entire */
	                        		                                                	/* original packet.                                                 */
	Sbhandle	               		mac_client_reassembly_buffer                    ;	/* Reassembly buffer for higher layer packets segmented in */
	                        		                                                	/* multiprotocol switches.                                 */
	WlanT_Mac_Frame_Type	   		fresp_to_send                                   ;	/* After receiving a frame station will determine that what response */
	                        		                                                	/* need to be sent.                                                  */
	double	                 		nav_duration                                    ;	/* Network allocation vector duration this an absolute time from the   */
	                        		                                                	/* beginning of the simulation. It will be considered as zero if it is */
	                        		                                                	/* set to the current time.                                            */
	Evhandle	               		nav_reset_evh                                   ;	/* Handle for the procedure interrupt that is scheduled to reset the */
	                        		                                                	/* NAV that is updated based on a received RTS, if no activity is    */
	                        		                                                	/* detected on the medium for acertain time as specified in section  */
	                        		                                                	/* 9.2.5.4 of the IEEE 802.11-1999 standard.                         */
	int	                    		rts_threshold                                   ;	/* Used in storing Rts threshold attribute */
	Boolean	                		duplicate_entry                                 ;	/* Flag for duplicate entry. Keeps track that whether the receive frame was */
	                        		                                                	/* a duplicate frame or not. This information is transmitted in ACK frame.  */
	WlanT_Mac_Frame_Type	   		expected_frame_type                             ;	/* Set the expected frame type needed in response */
	                        		                                                	/* to the transmitted frame                       */
	double	                 		backoff_slots                                   ;	/* Random number of backoff slots determined by */
	                        		                                                	/* a uniform distribution                       */
	OpT_Int64	              		remote_sta_addr                                 ;	/* Extracting remote station's address from */
	                        		                                                	/* the received packet                      */
	Stathandle	             		packet_load_handle                              ;	/* Recording number of packets received from the */
	                        		                                                	/* higher layer                                  */
	double	                 		intrpt_time                                     ;	/* Storing total backoff time when backoff duration is set */
	Packet *	               		wlan_transmit_frame_copy_ptr                    ;	/* Make copy of the transmit frame before transmission */
	Stathandle	             		backoff_slots_handle                            ;	/* Number of backoff slots before transmission. */
	int	                    		instrm_from_mac_if                              ;	/* Stream index coming from the higher layer MAC interface. */
	int	                    		outstrm_to_mac_if                               ;	/* Stream index connected to the higher layer MAC interface. */
	int	                    		num_fragments                                   ;	/* Number of data fragments that need to be transmitted */
	                        		                                                	/* for each data frame received from higher layer       */
	OpT_Packet_Size	        		remainder_size                                  ;	/* Size of the last data fragment */
	List*	                  		defragmentation_list_ptr                        ;	/* This buffer contains the fragments received from   */
	                        		                                                	/* remote station and maintains following information */
	                        		                                                	/* for each fragment:                                 */
	                        		                                                	/* 1. remote station address                          */
	                        		                                                	/* 2. time the last fragment was received             */
	                        		                                                	/* 3. reassembly buffer                               */
	WlanT_Mac_Flags*	       		wlan_flags                                      ;	/* This structure contains all the flags which the process    */
	                        		                                                	/* model used to convey information from one state to another */
	                        		                                                	/* for details check the header block where the structure is  */
	                        		                                                	/* defined.                                                   */
	OmsT_Aa_Address_Handle	 		oms_aa_handle                                   ;	/* used to obtain address handle to resolve wlan MAC address */
	double	                 		current_time                                    ;	/* Keeps track of the current simulation time at each interrupt */
	double	                 		rcv_idle_time                                   ;	/* Last simulation time when the receiver became idle again. */
	Pmohandle	              		hld_pmh                                         ;	/* This list stores the information of the received packet.  */
	                        		                                                	/* The information includes source station address, packet   */
	                        		                                                	/* sequence id, and packet fragment number. This             */
	                        		                                                	/* is used to keep track that whether the received packet is */
	                        		                                                	/* a duplicate of the previously received frame. If that's   */
	                        		                                                	/* the case then packet is simply discarded.                 */
	                        		                                                	/*                                                           */
	                        		                                                	/* Pool memory handle used to allocate memory for the data   */
	                        		                                                	/* received from the higher layer and inserted in the queue  */
	int	                    		max_backoff                                     ;	/* Maximum backoff value for picking random backoff interval */
	char	                   		current_state_name [32]                         ;	/* Keeping track of the current state of the station. */
	Stathandle	             		hl_packets_rcvd                                 ;	/* Monitor queue size as the packets arrive from higher layer. */
	Stathandle	             		media_access_delay                              ;	/* Keep tracks of the delay from the time the packet is received       */
	                        		                                                	/* from the higher layer to the time it is transmitted by the station. */
	Stathandle	             		ete_delay_handle                                ;	/* Handle for the end to end delay statistic that is recorded for   */
	                        		                                                	/* the packets that are accepted and forwarded to the higher layer. */
	Stathandle	             		global_ete_delay_handle                         ;	/* Handle for global end-to-end delay statistic. */
	Stathandle	             		global_throughput_handle                        ;	/* Handle for global WLAN throughput statistic. */
	Stathandle	             		global_load_handle                              ;	/* Handle for global WLAN load statistic. */
	Stathandle	             		global_buffer_drop_handle                       ;	/* Statistic handle for global dropped higher layer data due to full */
	                        		                                                	/* buffer.                                                           */
	Stathandle	             		global_retx_drop_handle                         ;	/* Statistic handle for global dropped higher layer data due to */
	                        		                                                	/* reaching the short or long retry limits.                     */
	Stathandle	             		global_mac_delay_handle                         ;	/* Handle for global media access delay statistic. */
	Stathandle	             		global_retrans_handle                           ;	/* Keep track of the global number of retransmissions before the packet was */
	                        		                                                	/* successfully transmitted.                                                */
	OmsT_Dim_Stat_Handle	   		global_network_load_handle                      ;	/* Handle for the network load statistic that is collected globally */
	                        		                                                	/* and dimensioned for each BSS.                                    */
	Stathandle	             		ctrl_traffic_rcvd_handle_inbits                 ;	/* Control Traffic (Rts,Cts or Ack) received by the station in bits */
	Stathandle	             		ctrl_traffic_sent_handle_inbits                 ;	/* Control Traffic (Rts,Cts or Ack) sent by the station in bits */
	Stathandle	             		ctrl_traffic_rcvd_handle                        ;	/* Control Traffic (Rts,Cts or Ack) received by the station in packets */
	Stathandle	             		ctrl_traffic_sent_handle                        ;	/* Control Traffic (Rts,Cts or Ack) sent by the station */
	Stathandle	             		mgmt_traffic_rcvd_handle_inbits                 ;	/* Management traffic received by the station in bits */
	Stathandle	             		mgmt_traffic_sent_handle_inbits                 ;	/* Management traffic sent by the station in bits */
	Stathandle	             		mgmt_traffic_rcvd_handle                        ;	/* Management traffic received by the station in packets */
	Stathandle	             		mgmt_traffic_sent_handle                        ;	/* Management traffic sent by the station in packets */
	Stathandle	             		data_traffic_rcvd_handle_inbits                 ;	/* Data Traffic received by the station in bits */
	Stathandle	             		data_traffic_sent_handle_inbits                 ;	/* Data Traffic sent  by the station in bits */
	Stathandle	             		data_traffic_rcvd_handle                        ;	/* Data Traffic received by the station */
	Stathandle	             		data_traffic_sent_handle                        ;	/* Data Traffic sent by the station */
	double	                 		sifs_time                                       ;	/* Read the SIFS time from the model attributes */
	double	                 		slot_time                                       ;	/* Read the Slot time from the model attributes */
	int	                    		cw_min                                          ;	/* Read the minimum contention window size from the model attribute. */
	int	                    		cw_max                                          ;	/* Read the maximum contention window size from the model attribute. */
	double	                 		difs_time                                       ;	/* DIFS interval is used by the stations to transmit data frames. */
	double	                 		plcp_overhead_control                           ;	/* Delay in seconds to transmit PLCP Preamble and PLCP Header at lowest */
	                        		                                                	/* mandatory data rate of the physical layer for WLAN control frames.   */
	double	                 		plcp_overhead_data                              ;	/* Delay in seconds to transmit PLCP Preamble and PLCP Header for WLAN */
	                        		                                                	/* data frames.                                                        */
	Stathandle	             		retrans_handle                                  ;	/* Keep track of the number of retransmissions before the packet was */
	                        		                                                	/* successfully transmitted.                                         */
	Stathandle	             		throughput_handle                               ;	/* Keep track of the number of data bits sent to the higher layer. */
	Stathandle	             		ap_conn_handle                                  ;	/* Statistic handle for recording AP connectivity of the MAC. */
	int	                    		long_retry_limit                                ;	/* This is the retry limit for the frames which are greater than or equal to */
	                        		                                                	/* Rts threshold.                                                            */
	int	                    		short_retry_limit                               ;	/* This is the retry limit for the frames which are less than or equal to */
	                        		                                                	/* Rts threshold.                                                         */
	int	                    		long_retry_count                                ;	/* Incremented when an ACK is not received for a data frame in spite of a */
	                        		                                                	/* successful RTS/CTS frame exchange.                                     */
	int	                    		short_retry_count                               ;	/* Incremented when a CTS is not received for a transmitted RTS, or when an */
	                        		                                                	/* ACK is not received for a data frame that is transmitted without any     */
	                        		                                                	/* RTS/CTS frame exchange.                                                  */
	WlanT_Mac_Frame_Type	   		last_frametx_type                               ;	/* This frame keeps track of the last transmitted frame that needs */
	                        		                                                	/* a frame response (like Cts or Ack). This is actually used when  */
	                        		                                                	/* there is a need for retransmission.                             */
	Evhandle	               		deference_evh                                   ;	/* Event handle that keeps track of the self interrupt due to Deference. */
	Evhandle	               		backoff_elapsed_evh                             ;	/* Event handle that keeps track of the self interrupt due to backoff. */
	Evhandle	               		frame_timeout_evh                               ;	/* Event handle that keeps track of the self interrupt due to frame timeout */
	                        		                                                	/* when the station is waiting for a response.                              */
	double	                 		eifs_time                                       ;	/* EIFS duration which is used when the station receives an erroneous frame */
	                        		                                                	/*                                                                          */
	int	                    		i_strm                                          ;	/* Keeping track of incoming packet stream from the lower layer. */
	Boolean	                		wlan_trace_active                               ;	/* Debugging/trace flag for all activities in this MAC. */
	OpT_Packet_Id	          		pkt_in_service                                  ;	/* Store packet id of the data packet in service. */
	Stathandle	             		bits_load_handle                                ;	/* Reporting the packet size arrived from higher layer. */
	int	                    		ap_flag                                         ;	/* Flag to read the attribute which indicates whether */
	                        		                                                	/* the station has Access point functional or not.    */
	Boolean	                		bss_flag                                        ;	/* Flag to check whether the network is configured for BSS or IBSS. */
	OpT_Int64	              		ap_mac_address                                  ;	/* The MAC address of the access point. */
	int	                    		hld_max_size                                    ;	/* This variable maintains the maximum size of the higher layer */
	                        		                                                	/* data buffer as specified by the user.                        */
	double	                 		max_receive_lifetime                            ;	/* Maximum time after the initial reception of the fragmented MSDU         */
	                        		                                                	/* after which further attempts to reassemble the MSDU will be terminated. */
	int	                    		accept_large_packets                            ;	/* Flag to accept packets larger than MAXMSDU size. */
	WlanT_Phy_Char_Code	    		phy_char_flag                                   ;	/* This variable stores the physical layer characteristic type information. */
	WlanT_Phy_Type	         		phy_type                                        ;	/* Type of the 802.11 standard that specifies the physical layer */
	                        		                                                	/* technology used by this MAC. Frequency hopping and infra-red  */
	                        		                                                	/* technologies are referred as 802.11b technologies.            */
	OpT_Packet_Size	        		total_hlpk_size                                 ;	/* Maintaining total size of the packets in higher layer queue. */
	int	                    		total_hlpk_num                                  ;	/* Maintaining total number of the packets in higher layer queue. */
	Stathandle	             		buffer_drop_pkts_handle                         ;	/* Keep track of the dropped packets in packets/sec by the higher */
	                        		                                                	/* layer queue due to the overflow of the buffer.                 */
	Stathandle	             		buffer_drop_bits_handle                         ;	/* Keep track of the dropped packets in bits/sec by the higher */
	                        		                                                	/* layer queue due to the overflow of the buffer.              */
	Stathandle	             		retx_drop_pkts_handle                           ;	/* Keep track of the dropped packets in packets/sec as a result of */
	                        		                                                	/* reaching short or long retry limit.                             */
	Stathandle	             		retx_drop_bits_handle                           ;	/* Keep track of the dropped packets in bits/sec as a result of */
	                        		                                                	/* reaching short or long retry limit.                          */
	Log_Handle	             		drop_pkt_log_handle                             ;	/* Logging information if the packet is dropped due to higher layer */
	                        		                                                	/* queue overflow.                                                  */
	Log_Handle	             		config_log_handle                               ;	/* Simulation log handle for configuration related messages. */
	int	                    		drop_pkt_entry_log_flag                         ;	/* This is to make sure that the entry is written only once. */
	double	                 		receive_time                                    ;	/* The arrival time of the packet that is currently handled. */
	Ici*	                   		llc_iciptr                                      ;	/* Sending ici to the bridge queue with final destination address. */
	double	                 		rx_power_threshold                              ;	/* Receiver power threshold for valid WLAN packets. */
	int	                    		bss_id                                          ;	/* When BSS_ID attribute is being used , this variable identifies  */
	                        		                                                	/* the BSS to which a node belongs. Otherwise, each subnet defines */
	                        		                                                	/* a BSS, and this state variable just retains its default value,  */
	                        		                                                	/* which is -1.                                                    */
	                        		                                                	/* CHANGED: BSS now stores the subnet object ID as the BSS ID      */
	int	                    		pcf_retry_count                                 ;	/* Incremented each time when a PCF frame was unsuccessful in transmission */
	int	                    		poll_fail_count                                 ;	/* This variable counts the number of times a poll is sent, and */
	                        		                                                	/* receives no response.                                        */
	int	                    		max_poll_fails                                  ;	/* The variable determines the maximum number of failed polls */
	                        		                                                	/* allowed before a station is skipped in sequence            */
	List*	                  		cfpd_list_ptr                                   ;	/* Higher layer data arrival queue or list for CFP */
	int	                    		pcf_queue_offset                                ;	/* Used in storing fragmentation threshold attribute */
	double	                 		beacon_int                                      ;	/* Beacon Interval in seconds */
	double	                 		beacon_tx_time                                  ;	/* This variable used only by APs when beacon transmission efficiency */
	                        		                                                	/* is enabled. It contains the TBTT of the last transmitted beacon,   */
	                        		                                                	/* or the next beacon whose transmission is already scheduled.        */
	Sbhandle	               		pcf_frag_buffer_ptr                             ;	/* PCF Fragmentation buffer used to store transmit packet fragments */
	Packet *	               		wlan_pcf_transmit_frame_copy_ptr                ;	/* Make separate copy of pcf transmit frames before transmission */
	int	                    		pcf_num_fragments                               ;	/* Number of data fragments that need to be transmitted */
	                        		                                                	/* for each pcf data frame received from higher layer   */
	OpT_Packet_Size	        		pcf_remainder_size                              ;	/* Size of the last PCF data fragment */
	OpT_Int64*	             		polling_list                                    ;	/* This structure is used to maintain the polling list for the PCF */
	                        		                                                	/*                                                                 */
	int	                    		poll_list_size                                  ;	/* This variable contains the size of the polling list */
	int	                    		poll_index                                      ;	/* This is the index into the polling list indicating the STA */
	                        		                                                	/* currently being polled.                                    */
	double	                 		pifs_time                                       ;	/* PIFS interval is used by the stations during CFP */
	Evhandle	               		cfp_end_evh                                     ;	/* Event handle that keeps track of self interrupt for end of CFP. */
	OpT_Packet_Id	          		pcf_pkt_in_service                              ;	/* Store packet id of the data packet in service. */
	int	                    		pcf_flag                                        ;	/* Flag to read the attribute which indicates whether */
	                        		                                                	/* the station has PCF functionality or not.          */
	Boolean	                		active_pc                                       ;	/* The variable indicates the presence of an active PC in the BSS */
	int	                    		cfp_prd                                         ;	/* This variable contain the number of beacon periods between */
	                        		                                                	/* Contention free periods                                    */
	int	                    		cfp_offset                                      ;	/* This variable is used to track the "phase" of the cfp    */
	                        		                                                	/* occurrence relative to time =0.  It indicates the number */
	                        		                                                	/* of beacon periods the first cfp is offset from time=0.   */
	double	                 		cfp_length                                      ;	/* This variable contains the default length in seconds of the */
	                        		                                                	/* CFP assuming the CFP starts at the TTBT.                    */
	OpT_Packet_Size	        		packet_size_dcf                                 ;	/* The full size of the DCF packet that is currently handled. */
	OpT_Packet_Size	        		packet_size_pcf                                 ;	/* The full size of the PCF packet that is currently handled. */
	double	                 		receive_time_dcf                                ;	/* The arrival time of the packet that is currently handled by the DCF. */
	double	                 		receive_time_pcf                                ;	/* The arrival time of the packet that is currently handled by the PCF. */
	Boolean	                		cfp_ap_medium_control                           ;	/* Indicates the CFP during which AP controls the medium. */
	int	                    		pcf_network                                     ;	/* Number of PCF enabled nodes in the network. */
	int	                    		beacon_tx_count                                 ;	/* The beacon transmission efficiency mode specified in terms of  */
	                        		                                                	/* number of beacons to be transmitted when it is essential to    */
	                        		                                                	/* transmit beacons. If the efficiency mode is disabled then this */
	                        		                                                	/* state variable will have the special value of "-1".            */
	int	                    		rem_beacon_tx                                   ;	/* This state variable used only by APs. It specifies how many more */
	                        		                                                	/* beacons need to be transmitted before beacon transmission is     */
	                        		                                                	/* stopped due to efficiency mode. If the efficiency mode is        */
	                        		                                                	/* disabled then this variable will have the special value "-1".    */
	int	                    		channel_count                                   ;	/* The number of operating WLAN radio channels, which     */
	                        		                                                	/* depends on the physical layer technology (i.e. 802.11a */
	                        		                                                	/* or 802.11/11b).                                        */
	int	                    		channel_num                                     ;	/* The number of the channel that is currently being used. */
	double	                 		first_chan_min_freq                             ;	/* The minimum frequency of the first one of the operating */
	                        		                                                	/* WLAN radio channels (in MHz).                           */
	double	                 		channel_bandwidth                               ;	/* The bandwidth of the operating WLAN radio channels in */
	                        		                                                	/* MHz.                                                  */
	double	                 		channel_spacing                                 ;	/* The spacing between two operating WLAN radio channels in */
	                        		                                                	/* MHz.                                                     */
	int	                    		eval_bss_id                                     ;	/* BSS ID of the BSS under evaluation. */
	WlanT_Roam_State_Info*	 		roam_state_ptr                                  ;	/* Roaming related information. */
	WlanT_Rx_State_Info*	   		rx_state_info_ptr                               ;	/* Receiver channel state information. */
	double	                 		ap_connectivity_check_interval                  ;	/* Time interval for periodic check on the connectivity from AP */
	double	                 		ap_connectivity_check_time                      ;	/* Time value indicating the next sim time to check the AP connectivity */
	Evhandle	               		ap_connectivity_check_evhndl                    ;	/* Event handle for the periodic AP connectivity check */
	WlanT_AP_Position_Info*			conn_ap_pos_info_ptr                            ;	/* Position information for the connected AP. */
	WlanT_Bss_Mapping_Info*			my_bss_info_ptr                                 ;	/* Handle to the information record of the current BSS */
	                        		                                                	/* including the list of STAs that belong to that BSS. */
	WlanT_Peer_Info*	       		ap_peer_info_ptr                                ;	/* Peer STA information record kept for the AP only by the non-AP */
	                        		                                                	/* STAs that belong to an infrastructure BSS.                     */
	PrgT_Bin_Hash_Table*	   		peer_info_hash_tbl                              ;	/* Hash table to access the information records of peer STAs quickly. */
	Boolean	                		debug_mode                                      ;	/* Flag that is set if the simulation is executed in debugging */
	                        		                                                	/* mode.                                                       */
	PrgT_Mutex*	            		mapping_info_mutex                              ;	/* Mutex used to serialize calling prg_mapping functions, which */
	                        		                                                	/* read/write global WLAN information, under multi-threaded     */
	                        		                                                	/* execution with multiple CPUs.                                */
	int	                    		data_packet_type                                ;	/* 自添加 */
	int	                    		data_packet_dest                                ;	/* 自添加 */
	int	                    		data_packet_final_dest                          ;	/* 自添加 */
	} wlan_mac_state;

#define intrpt_type             		op_sv_ptr->intrpt_type
#define intrpt_code             		op_sv_ptr->intrpt_code
#define my_address              		op_sv_ptr->my_address
#define my_objid                		op_sv_ptr->my_objid
#define my_node_objid           		op_sv_ptr->my_node_objid
#define my_subnet_objid         		op_sv_ptr->my_subnet_objid
#define tx_objid                		op_sv_ptr->tx_objid
#define txch_objid              		op_sv_ptr->txch_objid
#define rx_objid                		op_sv_ptr->rx_objid
#define rxch_objid              		op_sv_ptr->rxch_objid
#define own_process_record_handle		op_sv_ptr->own_process_record_handle
#define hld_list_ptr            		op_sv_ptr->hld_list_ptr
#define data_tx_rate            		op_sv_ptr->data_tx_rate
#define operational_speed       		op_sv_ptr->operational_speed
#define control_data_rate       		op_sv_ptr->control_data_rate
#define rcvd_frame_drate        		op_sv_ptr->rcvd_frame_drate
#define frag_threshold          		op_sv_ptr->frag_threshold
#define packet_seq_counter      		op_sv_ptr->packet_seq_counter
#define packet_seq_control      		op_sv_ptr->packet_seq_control
#define dcf_destination_addr    		op_sv_ptr->dcf_destination_addr
#define dcf_orig_source_addr    		op_sv_ptr->dcf_orig_source_addr
#define hl_protocol_dcf         		op_sv_ptr->hl_protocol_dcf
#define pcf_destination_addr    		op_sv_ptr->pcf_destination_addr
#define pcf_orig_source_addr    		op_sv_ptr->pcf_orig_source_addr
#define hl_protocol_pcf         		op_sv_ptr->hl_protocol_pcf
#define packet_frag_number      		op_sv_ptr->packet_frag_number
#define pcf_packet_frag_number  		op_sv_ptr->pcf_packet_frag_number
#define fragmentation_buffer_ptr		op_sv_ptr->fragmentation_buffer_ptr
#define common_rsmbuf_ptr       		op_sv_ptr->common_rsmbuf_ptr
#define mac_client_reassembly_buffer		op_sv_ptr->mac_client_reassembly_buffer
#define fresp_to_send           		op_sv_ptr->fresp_to_send
#define nav_duration            		op_sv_ptr->nav_duration
#define nav_reset_evh           		op_sv_ptr->nav_reset_evh
#define rts_threshold           		op_sv_ptr->rts_threshold
#define duplicate_entry         		op_sv_ptr->duplicate_entry
#define expected_frame_type     		op_sv_ptr->expected_frame_type
#define backoff_slots           		op_sv_ptr->backoff_slots
#define remote_sta_addr         		op_sv_ptr->remote_sta_addr
#define packet_load_handle      		op_sv_ptr->packet_load_handle
#define intrpt_time             		op_sv_ptr->intrpt_time
#define wlan_transmit_frame_copy_ptr		op_sv_ptr->wlan_transmit_frame_copy_ptr
#define backoff_slots_handle    		op_sv_ptr->backoff_slots_handle
#define instrm_from_mac_if      		op_sv_ptr->instrm_from_mac_if
#define outstrm_to_mac_if       		op_sv_ptr->outstrm_to_mac_if
#define num_fragments           		op_sv_ptr->num_fragments
#define remainder_size          		op_sv_ptr->remainder_size
#define defragmentation_list_ptr		op_sv_ptr->defragmentation_list_ptr
#define wlan_flags              		op_sv_ptr->wlan_flags
#define oms_aa_handle           		op_sv_ptr->oms_aa_handle
#define current_time            		op_sv_ptr->current_time
#define rcv_idle_time           		op_sv_ptr->rcv_idle_time
#define hld_pmh                 		op_sv_ptr->hld_pmh
#define max_backoff             		op_sv_ptr->max_backoff
#define current_state_name      		op_sv_ptr->current_state_name
#define hl_packets_rcvd         		op_sv_ptr->hl_packets_rcvd
#define media_access_delay      		op_sv_ptr->media_access_delay
#define ete_delay_handle        		op_sv_ptr->ete_delay_handle
#define global_ete_delay_handle 		op_sv_ptr->global_ete_delay_handle
#define global_throughput_handle		op_sv_ptr->global_throughput_handle
#define global_load_handle      		op_sv_ptr->global_load_handle
#define global_buffer_drop_handle		op_sv_ptr->global_buffer_drop_handle
#define global_retx_drop_handle 		op_sv_ptr->global_retx_drop_handle
#define global_mac_delay_handle 		op_sv_ptr->global_mac_delay_handle
#define global_retrans_handle   		op_sv_ptr->global_retrans_handle
#define global_network_load_handle		op_sv_ptr->global_network_load_handle
#define ctrl_traffic_rcvd_handle_inbits		op_sv_ptr->ctrl_traffic_rcvd_handle_inbits
#define ctrl_traffic_sent_handle_inbits		op_sv_ptr->ctrl_traffic_sent_handle_inbits
#define ctrl_traffic_rcvd_handle		op_sv_ptr->ctrl_traffic_rcvd_handle
#define ctrl_traffic_sent_handle		op_sv_ptr->ctrl_traffic_sent_handle
#define mgmt_traffic_rcvd_handle_inbits		op_sv_ptr->mgmt_traffic_rcvd_handle_inbits
#define mgmt_traffic_sent_handle_inbits		op_sv_ptr->mgmt_traffic_sent_handle_inbits
#define mgmt_traffic_rcvd_handle		op_sv_ptr->mgmt_traffic_rcvd_handle
#define mgmt_traffic_sent_handle		op_sv_ptr->mgmt_traffic_sent_handle
#define data_traffic_rcvd_handle_inbits		op_sv_ptr->data_traffic_rcvd_handle_inbits
#define data_traffic_sent_handle_inbits		op_sv_ptr->data_traffic_sent_handle_inbits
#define data_traffic_rcvd_handle		op_sv_ptr->data_traffic_rcvd_handle
#define data_traffic_sent_handle		op_sv_ptr->data_traffic_sent_handle
#define sifs_time               		op_sv_ptr->sifs_time
#define slot_time               		op_sv_ptr->slot_time
#define cw_min                  		op_sv_ptr->cw_min
#define cw_max                  		op_sv_ptr->cw_max
#define difs_time               		op_sv_ptr->difs_time
#define plcp_overhead_control   		op_sv_ptr->plcp_overhead_control
#define plcp_overhead_data      		op_sv_ptr->plcp_overhead_data
#define retrans_handle          		op_sv_ptr->retrans_handle
#define throughput_handle       		op_sv_ptr->throughput_handle
#define ap_conn_handle          		op_sv_ptr->ap_conn_handle
#define long_retry_limit        		op_sv_ptr->long_retry_limit
#define short_retry_limit       		op_sv_ptr->short_retry_limit
#define long_retry_count        		op_sv_ptr->long_retry_count
#define short_retry_count       		op_sv_ptr->short_retry_count
#define last_frametx_type       		op_sv_ptr->last_frametx_type
#define deference_evh           		op_sv_ptr->deference_evh
#define backoff_elapsed_evh     		op_sv_ptr->backoff_elapsed_evh
#define frame_timeout_evh       		op_sv_ptr->frame_timeout_evh
#define eifs_time               		op_sv_ptr->eifs_time
#define i_strm                  		op_sv_ptr->i_strm
#define wlan_trace_active       		op_sv_ptr->wlan_trace_active
#define pkt_in_service          		op_sv_ptr->pkt_in_service
#define bits_load_handle        		op_sv_ptr->bits_load_handle
#define ap_flag                 		op_sv_ptr->ap_flag
#define bss_flag                		op_sv_ptr->bss_flag
#define ap_mac_address          		op_sv_ptr->ap_mac_address
#define hld_max_size            		op_sv_ptr->hld_max_size
#define max_receive_lifetime    		op_sv_ptr->max_receive_lifetime
#define accept_large_packets    		op_sv_ptr->accept_large_packets
#define phy_char_flag           		op_sv_ptr->phy_char_flag
#define phy_type                		op_sv_ptr->phy_type
#define total_hlpk_size         		op_sv_ptr->total_hlpk_size
#define total_hlpk_num          		op_sv_ptr->total_hlpk_num
#define buffer_drop_pkts_handle 		op_sv_ptr->buffer_drop_pkts_handle
#define buffer_drop_bits_handle 		op_sv_ptr->buffer_drop_bits_handle
#define retx_drop_pkts_handle   		op_sv_ptr->retx_drop_pkts_handle
#define retx_drop_bits_handle   		op_sv_ptr->retx_drop_bits_handle
#define drop_pkt_log_handle     		op_sv_ptr->drop_pkt_log_handle
#define config_log_handle       		op_sv_ptr->config_log_handle
#define drop_pkt_entry_log_flag 		op_sv_ptr->drop_pkt_entry_log_flag
#define receive_time            		op_sv_ptr->receive_time
#define llc_iciptr              		op_sv_ptr->llc_iciptr
#define rx_power_threshold      		op_sv_ptr->rx_power_threshold
#define bss_id                  		op_sv_ptr->bss_id
#define pcf_retry_count         		op_sv_ptr->pcf_retry_count
#define poll_fail_count         		op_sv_ptr->poll_fail_count
#define max_poll_fails          		op_sv_ptr->max_poll_fails
#define cfpd_list_ptr           		op_sv_ptr->cfpd_list_ptr
#define pcf_queue_offset        		op_sv_ptr->pcf_queue_offset
#define beacon_int              		op_sv_ptr->beacon_int
#define beacon_tx_time          		op_sv_ptr->beacon_tx_time
#define pcf_frag_buffer_ptr     		op_sv_ptr->pcf_frag_buffer_ptr
#define wlan_pcf_transmit_frame_copy_ptr		op_sv_ptr->wlan_pcf_transmit_frame_copy_ptr
#define pcf_num_fragments       		op_sv_ptr->pcf_num_fragments
#define pcf_remainder_size      		op_sv_ptr->pcf_remainder_size
#define polling_list            		op_sv_ptr->polling_list
#define poll_list_size          		op_sv_ptr->poll_list_size
#define poll_index              		op_sv_ptr->poll_index
#define pifs_time               		op_sv_ptr->pifs_time
#define cfp_end_evh             		op_sv_ptr->cfp_end_evh
#define pcf_pkt_in_service      		op_sv_ptr->pcf_pkt_in_service
#define pcf_flag                		op_sv_ptr->pcf_flag
#define active_pc               		op_sv_ptr->active_pc
#define cfp_prd                 		op_sv_ptr->cfp_prd
#define cfp_offset              		op_sv_ptr->cfp_offset
#define cfp_length              		op_sv_ptr->cfp_length
#define packet_size_dcf         		op_sv_ptr->packet_size_dcf
#define packet_size_pcf         		op_sv_ptr->packet_size_pcf
#define receive_time_dcf        		op_sv_ptr->receive_time_dcf
#define receive_time_pcf        		op_sv_ptr->receive_time_pcf
#define cfp_ap_medium_control   		op_sv_ptr->cfp_ap_medium_control
#define pcf_network             		op_sv_ptr->pcf_network
#define beacon_tx_count         		op_sv_ptr->beacon_tx_count
#define rem_beacon_tx           		op_sv_ptr->rem_beacon_tx
#define channel_count           		op_sv_ptr->channel_count
#define channel_num             		op_sv_ptr->channel_num
#define first_chan_min_freq     		op_sv_ptr->first_chan_min_freq
#define channel_bandwidth       		op_sv_ptr->channel_bandwidth
#define channel_spacing         		op_sv_ptr->channel_spacing
#define eval_bss_id             		op_sv_ptr->eval_bss_id
#define roam_state_ptr          		op_sv_ptr->roam_state_ptr
#define rx_state_info_ptr       		op_sv_ptr->rx_state_info_ptr
#define ap_connectivity_check_interval		op_sv_ptr->ap_connectivity_check_interval
#define ap_connectivity_check_time		op_sv_ptr->ap_connectivity_check_time
#define ap_connectivity_check_evhndl		op_sv_ptr->ap_connectivity_check_evhndl
#define conn_ap_pos_info_ptr    		op_sv_ptr->conn_ap_pos_info_ptr
#define my_bss_info_ptr         		op_sv_ptr->my_bss_info_ptr
#define ap_peer_info_ptr        		op_sv_ptr->ap_peer_info_ptr
#define peer_info_hash_tbl      		op_sv_ptr->peer_info_hash_tbl
#define debug_mode              		op_sv_ptr->debug_mode
#define mapping_info_mutex      		op_sv_ptr->mapping_info_mutex
#define data_packet_type        		op_sv_ptr->data_packet_type
#define data_packet_dest        		op_sv_ptr->data_packet_dest
#define data_packet_final_dest  		op_sv_ptr->data_packet_final_dest

/* These macro definitions will define a local variable called	*/
/* "op_sv_ptr" in each function containing a FIN statement.	*/
/* This variable points to the state variable data structure,	*/
/* and can be used from a C debugger to display their values.	*/
#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE
#define FIN_PREAMBLE_DEC	wlan_mac_state *op_sv_ptr;
#define FIN_PREAMBLE_CODE	\
		op_sv_ptr = ((wlan_mac_state *)(OP_SIM_CONTEXT_PTR->_op_mod_state_ptr));


/* Function Block */

#if !defined (VOSD_NO_FIN)
enum { _op_block_origin = __LINE__ + 2};
#endif

static void
wlan_mac_sv_init ()
	{
	Objid					mac_params_comp_attr_objid;
	Objid					params_attr_objid;
	Objid					pcf_params_comp_attr_objid;
	Objid					subpcf_params_attr_objid;	
	Objid					chann_objid;
	int						num_chann;
	Boolean					data_rate_11b;
	double					tx_power;
	int						i;
	Objid					statwire_objid;
	int						num_statwires;
	double					threshold;
	double					rx_power_threshold_dbm;
	void*                   temp_ptr;
	int						roaming_cap_flag, cts_to_self_flag;
	Evhandle				invalid_evh = OPC_EV_HANDLE_INVALID;
	char					stat_info [16];
	int						integer_mac_address = -1;
	
	/**	1. Initialize state variables.						**/
	/** 2. Read model attribute values in variables.	    **/
	/** 3. Create global lists								**/
	/** 4. Register statistics handlers						**/
	FIN (wlan_mac_sv_init ());

	/* object id of the surrounding processor.				*/
	my_objid = op_id_self ();

	/* Obtain the node's object identifier					*/
	my_node_objid = op_topo_parent (my_objid);

	/* Obtain subnet objid.									*/
	my_subnet_objid = op_topo_parent (my_node_objid);

	/* Obtain the values assigned to the various attributes	*/
	op_ima_obj_attr_get (my_objid, "Wireless LAN Parameters", &mac_params_comp_attr_objid);
    params_attr_objid = op_topo_child (mac_params_comp_attr_objid, OPC_OBJTYPE_GENERIC, 0);

	/* Determine the assigned MAC address.					*/
	op_ima_obj_attr_get (my_objid, "Address", &integer_mac_address);
	my_address = integer_mac_address;
	
	/* Obtain an address handle for resolving WLAN MAC addresses.				*/
	oms_aa_handle = oms_aa_address_handle_get ("MAC Addresses", "Address");

 	/* Obtain the BSS_Id attribute to determine if BSS based network is used	*/ 
	op_ima_obj_attr_get (params_attr_objid, "BSS Identifier", &bss_id);
	
	/* Register the log handles and related flags.								*/
	config_log_handle	= op_prg_log_handle_create (OpC_Log_Category_Configuration, "Wireless LAN", "MAC Configuration", 128);
	drop_pkt_log_handle	= op_prg_log_handle_create (OpC_Log_Category_Protocol,      "Wireless LAN", "Data packet Drop",  128);
    drop_pkt_entry_log_flag = 0;

	/* Update the global variable if this is the first node to come up. If not the	*/ 
	/* first node, then check for mismatches. A subnet can be a traditional subnet	*/
	/* (i.e. a subnet with one BSS, this is the existing model) or a  BSS based		*/
	/* subnet where for every node the attribute BSS_Id is set to indicate to which */
	/* BSS a node belongs. If the global is set to traditional subnet and the this	*/
	/* node has its BSS_Id attribute set then log a warning message and recover		*/
	/* by considering the BSS_Id attribute setting as not used. If the global is	*/
	/* set to BSS based subnet and this node is not using its BSS_Id attribute 		*/
	/* then log an error message and stop the simulation.					 		*/
	if (bss_id_type == WlanC_Not_Set)
		{
		if (bss_id ==  WLANC_BSSID_NOT_USED)
			{
			bss_id_type = WlanC_Entire_Subnet ;
			}
		else
			{
			bss_id_type = WlanC_Bss_Divided_Subnet ;
			}
		}

	/* Configuration error checking */
	if (bss_id_type == WlanC_Entire_Subnet && bss_id != WLANC_BSSID_NOT_USED)
		{
		/* Recoverable mismatch, log warning and continue by enforcing			*/
		/* traditional subnet, i.e. force the bss_id variable to not used.		*/

		/* Write the warning message.											*/
		op_prg_log_entry_write (config_log_handle,
			"WARNING:\n"
			" A node with an explicit BSS\n"
			" assignment was found in a network\n"
			" containing WLAN nodes with auto-\n"
			" assigned BSS identifiers.\n"
			"ACTION:\n"
			" The BSS identifier is set to\n"
			" the default value \"Auto Assigned\".\n"
			"CAUSE:\n"
			" There are some nodes in the\n"
			" network which have their BSS\n"
			" identifiers set explicitly\n"
			" while the others have the\n"
			" default \"Auto Assigned\" setting.\n"
			"SUGGESTION:\n"
			" Ensure that all nodes have their\n"
			" BSS identifier either set to the\n"
			" default value or explicitly\n"
			" assigned.\n");
		}
	else if (bss_id_type == WlanC_Bss_Divided_Subnet && bss_id == WLANC_BSSID_NOT_USED)
		{
		/* Unrecoverable error-- not all BSS IDs have been configured. Cannot 	*/
		/* what the BSS ID should be, hence terminate.							*/

		wlan_error_print ("BSS ID not set in a node which belongs to a network in which some BSS IDs are set", 
						  "Please set a non-default BSS ID on all nodes in the network", OPC_NIL);
		}

	/* Use the subnet ID as the BSS ID if it is set to "Auto Assigned".			*/
	if (bss_id_type == WlanC_Entire_Subnet)
		{
		bss_id = my_subnet_objid;
		}

   	/* Get model attributes.													*/
	op_ima_obj_attr_get (params_attr_objid, "Data Rate", &data_tx_rate);
	op_ima_obj_attr_get (params_attr_objid, "Fragmentation Threshold", &frag_threshold);
	op_ima_obj_attr_get (params_attr_objid, "Rts Threshold", &rts_threshold);
	op_ima_obj_attr_get (params_attr_objid, "Short Retry Limit", &short_retry_limit);
	op_ima_obj_attr_get (params_attr_objid, "Long Retry Limit", &long_retry_limit);
	op_ima_obj_attr_get (params_attr_objid, "Access Point Functionality", &ap_flag);
	op_ima_obj_attr_get (params_attr_objid, "AP Beacon Interval", &beacon_int);
	op_ima_obj_attr_get (params_attr_objid, "Buffer Size", &hld_max_size);
	op_ima_obj_attr_get (params_attr_objid, "Max Receive Lifetime", &max_receive_lifetime);
	op_ima_obj_attr_get (params_attr_objid, "Large Packet Processing", &accept_large_packets);

	/* If specified, convert the fragmentation and RTS thresholds to bits.		*/
	if (frag_threshold != -1)
		frag_threshold *= 8;
	if (rts_threshold != -1)
		rts_threshold *= 8;
	
	/* Check whether beacon transmission efficiency is enabled.					*/
	op_ima_sim_attr_get (OPC_IMA_INTEGER, "WLAN Beacon Transmission Count", &beacon_tx_count);

	/* Extract beacon and PCF parameters.										*/
	op_ima_obj_attr_get (params_attr_objid, "PCF Parameters", &pcf_params_comp_attr_objid);
	subpcf_params_attr_objid = op_topo_child (pcf_params_comp_attr_objid, OPC_OBJTYPE_GENERIC, 0);
	op_ima_obj_attr_get (subpcf_params_attr_objid, "PCF Functionality", &pcf_flag);	

	/* Check if there is an active AP controlling the medium during the CFP.	*/
	if ((ap_flag == OPC_BOOLINT_ENABLED) && (pcf_flag == OPC_BOOLINT_ENABLED))
		{
		/* We will also operate as a PC.										*/
		active_pc = OPC_TRUE;
		
		/* Read the value of the parameters used by the PC.						*/
		op_ima_obj_attr_get (subpcf_params_attr_objid, "CFP Beacon Multiple", &cfp_prd);	
		op_ima_obj_attr_get (subpcf_params_attr_objid, "CFP Offset", &cfp_offset);	
		op_ima_obj_attr_get (subpcf_params_attr_objid, "CFP Interval", &cfp_length);	
		op_ima_obj_attr_get (subpcf_params_attr_objid, "Max Failed Polls", &max_poll_fails);	
		}
	else
		active_pc = OPC_FALSE;

	/* Load the appropriate physical layer characteristics.					*/	
	op_ima_obj_attr_get (params_attr_objid, "Physical Characteristics", &phy_char_flag);

	/* Obtain the receiver valid packet power threshold value used by the	*/
	/* statwires from the receiver into the MAC module.						*/
	op_ima_obj_attr_get (params_attr_objid, "Packet Reception-Power Threshold", &rx_power_threshold_dbm);
	
	/* Convert the power threshold (receiver sensitivity) value from dBm to	*/
	/* Watts.																*/
	rx_power_threshold = pow (10.0, rx_power_threshold_dbm / 10.0) / 1000.0;
	
	/* Get the transmission power value in Watts.							*/
	op_ima_obj_attr_get (params_attr_objid, "Transmit Power", &tx_power);	
	
	/* Make sure that the configured data rate is compatible with the		*/
	/* physical layer technology.											*/
	data_rate_11b = (data_tx_rate < 6000000.0 || data_tx_rate == 11000000.0) ? OPC_TRUE : OPC_FALSE;
	if ((phy_char_flag == WlanC_OFDM_11a && data_rate_11b) || 
		(phy_char_flag != WlanC_OFDM_11a && phy_char_flag != WlanC_ERP_OFDM_11g && !data_rate_11b)) 
		{
		/* Mismatching data rate <-> physical layer technology. Write a 	*/
		/* simulation log message.											*/
		op_prg_log_entry_write (config_log_handle,
			"ERROR:\n"
			" The WLAN data rate %.1f Mbps configured \n"
			" for this node cannot be supported with \n"
			" the specified physical layer technology.\n"
			"\n"	
			"REMEDIAL ACTION:\n"
			" The operational data rate is set to\n"
			" %.1f Mbps to be compatible with the\n"
			" physical layer configuration.\n"
			"\n"	
			"SUGGESTION:\n"
			" Ensure that the \"Wireless LAN Parameters ->\n"
			" Physical Characteristics\" attribute is set\n"
			" properly to support the configured WLAN data\n"
			" rate according to the corresponding WLAN\n"
			" standard.\n", data_tx_rate / 1000000.0, 
			((phy_char_flag == WlanC_OFDM_11a) ? WLANC_11a_MIN_MANDATORY_DRATE : WLANC_11b_MIN_MANDATORY_DRATE) / 1000000.0);
		
		/* Set the data rate to the minimum data rate of the specified		*/
		/* physical layer technology.										*/
		data_tx_rate = (phy_char_flag == WlanC_OFDM_11a) ? WLANC_11a_MIN_MANDATORY_DRATE : WLANC_11b_MIN_MANDATORY_DRATE;
		}
		
	/* Initialize our current data transmission rate with the configured	*/
	/* value.																*/
	operational_speed = data_tx_rate;
	
	/* Based on physical characteristics settings set appropriate values to	*/
	/* the variables.														*/
	switch (phy_char_flag)
		{
		case WlanC_Frequency_Hopping:
			{
			/* Slot duration in terms of seconds.							*/
			slot_time = 50E-06;

			/* Short interframe gap in terms of seconds.					*/
			sifs_time = 28E-06;
			
			/* PLCP overheads, which include the preamble and header, in	*/
			/* terms of seconds.											*/
			plcp_overhead_control = WLANC_PLCP_OVERHEAD_FHSS;
			plcp_overhead_data    = WLANC_PLCP_OVERHEAD_FHSS;
			
			/* Minimum contention window size for selecting backoff slots.	*/
			cw_min = 15;

			/* Maximum contention window size for selecting backoff slots.	*/
			cw_max = 1023;
			
			/* Set the PHY standard as 11b for the technologies specified	*/
			/* in 802.11 and 802.11b.										*/
			phy_type = WlanC_11b_PHY;
			break;
			}

		case WlanC_Direct_Sequence:
			{
			/* Slot duration in terms of seconds.							*/
			slot_time = 20E-06;

			/* Short interframe gap in terms of seconds.					*/
			sifs_time = 10E-06;

			/* PLCP overheads, which include the preamble and header, in	*/
			/* terms of seconds.											*/
			plcp_overhead_control = WLANC_PLCP_OVERHEAD_DSSS_LONG;
			plcp_overhead_data    = WLANC_PLCP_OVERHEAD_DSSS_LONG;
			
			/* Minimum contention window size for selecting backoff slots.	*/
			cw_min = 31;

			/* Maximum contention window size for selecting backoff slots.	*/
			cw_max = 1023;
			
			/* Set the PHY standard as 11b for the technologies specified	*/
			/* in 802.11 and 802.11b.										*/
			phy_type = WlanC_11b_PHY;
			break;
			}

		case WlanC_Infra_Red:
			{
			/* Slot duration in terms of seconds.							*/
			slot_time = 8E-06;

			/* Short interframe gap in terms of seconds (this is changed	*/
			/* from 7 usec (IEEE 802.11 1997 Edition) to 10 usec (IEEE		*/
			/* 802.11 1999 Edition).										*/
			sifs_time = 10E-06;

			/* PLCP overheads, which include the preamble and header, in	*/
			/* terms of seconds. Infra-red supports transmission of parts	*/
			/* of the PLCP header at the regular data transmission rate,	*/
			/* which can be higher than mandatory lowest data rate.			*/
			plcp_overhead_control = WLANC_PLCP_OVERHEAD_IR_1MBPS;
			plcp_overhead_data    = (data_tx_rate == 1000000.0) ? WLANC_PLCP_OVERHEAD_IR_1MBPS : WLANC_PLCP_OVERHEAD_IR_2MBPS;
	  
			/* Minimum contention window size for selecting backoff slots.	*/
			cw_min = 63;

			/* Maximum contention window size for selecting backoff slots.	*/
			cw_max = 1023;
			
			/* Set the PHY standard as 11b for the technologies specified	*/
			/* in 802.11 and 802.11b.										*/
			phy_type = WlanC_11b_PHY;
			break;
			}

		case WlanC_OFDM_11a:
			{
			/* Slot duration in terms of seconds.							*/
			slot_time = 9E-06;

			/* Short interframe gap in terms of seconds.					*/
			sifs_time = 16E-06;

			/* PLCP overheads, which include the preamble and header, in	*/
			/* terms of seconds. For OFDM (11a), these overheads don't		*/
			/* include the bits of the PLCP header fields "SERVICE", "Tail"	*/
			/* and "Pad". The overhead of these fields will be computed		*/
			/* when the size of the MAC layer's packet (MPDU) is known.		*/
			plcp_overhead_control = WLANC_PLCP_OVERHEAD_OFDM;
			plcp_overhead_data    = WLANC_PLCP_OVERHEAD_OFDM;
	  
			/* Minimum contention window size for selecting backoff slots.	*/
			cw_min = 15;

			/* Maximum contention window size for selecting backoff slots.	*/
			cw_max = 1023;
			
			/* Set the PHY standard.										*/
			phy_type = WlanC_11a_PHY;
			break;
			}

		case WlanC_ERP_OFDM_11g:
			{			
			/* Set the slot time to 9E-6 seconds (short) initially. We will	*/
			/* increase it to 20 usec (long) if we detect that we operate	*/
			/* in an IBSS or in a BSS that also has non-ERP STAs associated.*/
			slot_time = 9E-06;

			/* Short interframe gap in terms of seconds.					*/
			sifs_time = 10E-06;

			/* PLCP overheads, which include the preamble and header, in	*/
			/* terms of seconds. Assume ERP-OFDM preamble. We will adjust	*/
			/* the overhead amount if regular long or short DSSS preambles	*/
			/* are used.													*/
			plcp_overhead_control = WLANC_PLCP_OVERHEAD_OFDM;
			plcp_overhead_data    = WLANC_PLCP_OVERHEAD_OFDM;
			
			/* Minimum contention window size for selecting backoff slots.	*/
			/* Initially we pick the lower CWmin and increase it to 31 if	*/
			/* we operate in an IBSS containing some non-ERP STAs or if we	*/
			/* are associated with a non-ERP AP.							*/
			cw_min = 15;

			/* Maximum contention window size for selecting backoff slots.	*/
			cw_max = 1023;
			
			/* Set the PHY standard.										*/
			phy_type = WlanC_11g_PHY;
			break;
			}
			
		default:
			{
			wlan_error_print ("Unexpected Physical Layer Characteristic encountered.", OPC_NIL, OPC_NIL);
			break;
			}
		}

	/* Initialize the state variables whose values are based on whether		*/
	/* this MAC is operating with 802.11a, 802.11/802.11b or 802.11g PHY.	*/
	if (phy_type == WlanC_11a_PHY)
		{
		/* Set the data rate (in bps) used for the transmission of control	*/
		/* frames. Pick the highest mandatory data rate that is equal to or	*/
		/* lower than the data rate specified for data transmissions.		*/
		for (i = 0; data_tx_rate < WLANC_11a_MANDATORY_DRATE_ARRAY [i]; i++);
		control_data_rate = WLANC_11a_MANDATORY_DRATE_ARRAY [i];
		
		/* Initialize the state variables related to channelization, which	*/
		/* are mainly used during roaming (handover) procedures.			*/
		channel_count       = WLANC_11a_OPER_CHNL_COUNT;
		first_chan_min_freq = WLANC_11a_FIRST_CHNL_MIN_FREQ;
		channel_bandwidth   = WLANC_11a_CHNL_BANDWIDTH;
		channel_spacing     = WLANC_11a_CHNL_SPACING;
		}
	else
		{
		/* The MAC is operating in either 802.11/802.11b or 802.11g mode.	*/
		
		/* Initialize the state variables related to channelization, which	*/
		/* are mainly used during roaming (handover) procedures.			*/
		channel_count       = WLANC_11b_OPER_CHNL_COUNT;
		first_chan_min_freq = WLANC_11b_FIRST_CHNL_MIN_FREQ;
		channel_bandwidth   = WLANC_11b_CHNL_BANDWIDTH;
		channel_spacing     = WLANC_11b_CHNL_SPACING;
		
		/* Set the data rate (in bps) used for the transmission of control	*/
		/* frames.															*/
		if (phy_type == WlanC_11g_PHY)
			{
			/* Choose the highest mandatory data rate that is equal to or	*/
			/* lower than the data rate specified for data transmissions.	*/
			for (i = 0; data_tx_rate < WLANC_11g_MANDATORY_DRATE_ARRAY [i]; i++);
			control_data_rate = WLANC_11g_MANDATORY_DRATE_ARRAY [i];
			}
		else
			{
			/* Choose the lowest 802.11/11b mandatory data rate.			*/
			control_data_rate = WLANC_11b_MIN_MANDATORY_DRATE;
			}
		}

	/* Allocating memory for the flags used in this process model. 			*/
	wlan_flags = (WlanT_Mac_Flags *) op_prg_mem_alloc (sizeof (WlanT_Mac_Flags));

	/* Initially resetting all the flags.									*/
	wlan_flags->data_frame_to_send 	= OPC_FALSE;
	wlan_flags->backoff_flag       	= OPC_FALSE;
	wlan_flags->rts_sent		   	= OPC_FALSE;
	wlan_flags->rcvd_bad_packet		= OPC_FALSE;
	wlan_flags->bad_packet_dropped	= OPC_FALSE;
	wlan_flags->receiver_busy		= OPC_FALSE;
	wlan_flags->transmitter_busy	= OPC_FALSE;
	wlan_flags->gateway_flag		= OPC_FALSE;
	wlan_flags->bridge_flag			= OPC_FALSE;
	wlan_flags->wait_eifs_dur		= OPC_FALSE;
	wlan_flags->immediate_xmt		= OPC_FALSE;
	wlan_flags->forced_bk_end  	    = OPC_FALSE;
	wlan_flags->cw_required			= OPC_FALSE;
	wlan_flags->perform_cw			= OPC_FALSE;
	wlan_flags->nav_updated			= OPC_FALSE;
	wlan_flags->collision			= OPC_FALSE;
	wlan_flags->scanning			= OPC_FALSE;

	wlan_flags->duration_zero		= OPC_FALSE;
	wlan_flags->ignore_busy			= OPC_FALSE;
	wlan_flags->tx_beacon			= OPC_FALSE;
	wlan_flags->tx_cf_end			= OPC_FALSE;
	wlan_flags->pcf_active			= OPC_FALSE;
	wlan_flags->polled				= OPC_FALSE;
	wlan_flags->more_data			= OPC_FALSE;
	wlan_flags->more_frag			= OPC_FALSE;
	wlan_flags->pcf_side_traf		= OPC_FALSE;
	wlan_flags->active_poll			= OPC_FALSE;

	wlan_flags->non_erp_present		= OPC_FALSE;
	wlan_flags->wait_signal_ext		= OPC_FALSE;
	wlan_flags->rcvd_bad_cts 		= OPC_FALSE;
	wlan_flags->pcf_lowered_drate	= OPC_FALSE;

	/* Set the flag corresponding to optional 802.11g protection mechanism	*/
	/* "CTS-to-self" based on user's configuration.							*/
	op_ima_obj_attr_get (params_attr_objid, "CTS-to-self Option", &cts_to_self_flag);
	wlan_flags->cts_to_self = (cts_to_self_flag == OPC_BOOLINT_ENABLED) ? OPC_TRUE : OPC_FALSE;

	/* If the BSS IDs are auto-assigned then add the BSS ID into the 		*/
	/* physical layer technology specific BSS ID list, which is later going	*/
	/* to be used while selecting channels for BSSs.						*/
	if (bss_id_type == WlanC_Entire_Subnet)
		{
		wlan_bss_id_list_manage (bss_id, phy_type, "add");
		}

	/* By default stations are configured for IBSS unless an Access Point	*/
	/* is found, then the network will have an infrastructure BSS			*/
	/* configuration.														*/
	bss_flag = OPC_FALSE;

	/* Compute the values of various interframe spacing parameters based on	*/
	/* the timing relations specified in section 9.2.10 of the IEEE 802.11-	*/
	/* 1999 standard.														*/
	
	/* Computing DIFS interval which is interframe gap between successive	*/
	/* frame transmissions.													*/
	difs_time = sifs_time + 2 * slot_time;

	/* Compute the EIFS duration, which is DIFS + SIFS + "transmission time	*/
	/* of an ACK frame". While computing the the ACK transmission time, use	*/
	/* the lowest mandatory data rate of the PHY as stated in the standard. */
	if (phy_type == WlanC_11a_PHY)
		eifs_time = difs_time + sifs_time + TXTIME_CTRL_DR (WLANC_ACK_LENGTH, WLANC_11a_MIN_MANDATORY_DRATE);
	else
		eifs_time = difs_time + sifs_time + TXTIME_CTRL_DR (WLANC_ACK_LENGTH, WLANC_11b_MIN_MANDATORY_DRATE);
	
	/* PIFS duration is used by the AP operating under PCF to gain priority	*/
	/* to access the medium.												*/
	pifs_time = sifs_time + slot_time;
	
	/* Creating list to store data arrived from higher layer.				*/	
	hld_list_ptr = op_prg_list_create ();

	/* If the station is an AP, and PCF supported, create separate PCF		*/
	/* queue list for higher layer. 										*/
	if ((ap_flag == OPC_BOOLINT_ENABLED) && (pcf_flag == OPC_BOOLINT_ENABLED))
		cfpd_list_ptr = op_prg_list_create ();
	else
		cfpd_list_ptr = OPC_NIL;
	
	/* Initialize segmentation and reassembly buffers.						*/
	defragmentation_list_ptr 	 = op_prg_list_create ();
	fragmentation_buffer_ptr 	 = op_sar_buf_create (OPC_SAR_BUF_TYPE_SEGMENT,    OPC_SAR_BUF_OPT_PK_BNDRY);
	common_rsmbuf_ptr        	 = op_sar_buf_create (OPC_SAR_BUF_TYPE_REASSEMBLY, OPC_SAR_BUF_OPT_DEFAULT);
	mac_client_reassembly_buffer = op_sar_buf_create (OPC_SAR_BUF_TYPE_REASSEMBLY, OPC_SAR_BUF_OPT_DEFAULT);
	
	/* Initialize PCF segmentation and reassembly buffer. (only used by AP)	*/
	pcf_frag_buffer_ptr = op_sar_buf_create (OPC_SAR_BUF_TYPE_SEGMENT, OPC_SAR_BUF_OPT_PK_BNDRY);

	/* Set a discard timer for the reassembly buffer of the MAC client		*/
	/* packets. Note that this is to reassemble the MSDUs that are segments	*/
	/* themselves, not to reassemble the MSDUs that are fragmented as a		*/
	/* result of being larger than the configured WLAN fragmentation		*/
	/* threshold. Hence, set the duration of the discard timer to the twice	*/
	/* of the maximum receive lifetime of the MSDU fragments.				*/
	op_sar_buf_options_set (mac_client_reassembly_buffer, OPC_SAR_BUF_OPT_FLUSH_TIMED, 2.0 * max_receive_lifetime);

	/* Registering local statistics.										*/
	packet_load_handle				= op_stat_reg ("Wireless Lan.Load (packets/sec)", 				                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);
	bits_load_handle				= op_stat_reg ("Wireless Lan.Load (bits/sec)", 					                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);
	hl_packets_rcvd					= op_stat_reg ("Wireless Lan.Queue Size (packets)", 		                     	 OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);
	backoff_slots_handle			= op_stat_reg ("Wireless Lan.Backoff Slots (slots)", 			                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);
	data_traffic_sent_handle 		= op_stat_reg ("Wireless Lan.Data Traffic Sent (packets/sec)", 	                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);	
	data_traffic_rcvd_handle		= op_stat_reg ("Wireless Lan.Data Traffic Rcvd (packets/sec)", 	                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL); 
	data_traffic_sent_handle_inbits	= op_stat_reg ("Wireless Lan.Data Traffic Sent (bits/sec)", 	                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);
	data_traffic_rcvd_handle_inbits	= op_stat_reg ("Wireless Lan.Data Traffic Rcvd (bits/sec)", 	                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);
	ctrl_traffic_sent_handle	 	= op_stat_reg ("Wireless Lan.Control Traffic Sent (packets/sec)",                    OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);
	ctrl_traffic_rcvd_handle		= op_stat_reg ("Wireless Lan.Control Traffic Rcvd (packets/sec)",                    OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL); 
	ctrl_traffic_sent_handle_inbits	= op_stat_reg ("Wireless Lan.Control Traffic Sent (bits/sec)",                       OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);
	ctrl_traffic_rcvd_handle_inbits	= op_stat_reg ("Wireless Lan.Control Traffic Rcvd (bits/sec)", 	                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL); 
	mgmt_traffic_rcvd_handle		= op_stat_reg ("Wireless Lan.Management Traffic Rcvd (packets/sec)",                 OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL); 
	mgmt_traffic_rcvd_handle_inbits	= op_stat_reg ("Wireless Lan.Management Traffic Rcvd (bits/sec)", 	                 OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL); 
	buffer_drop_pkts_handle       	= op_stat_reg ("Wireless Lan.Data Dropped (Buffer Overflow) (packets/sec)",          OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL); 
	buffer_drop_bits_handle       	= op_stat_reg ("Wireless Lan.Data Dropped (Buffer Overflow) (bits/sec)",             OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL); 
	retx_drop_pkts_handle       	= op_stat_reg ("Wireless Lan.Data Dropped (Retry Threshold Exceeded) (packets/sec)", OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL); 
	retx_drop_bits_handle       	= op_stat_reg ("Wireless Lan.Data Dropped (Retry Threshold Exceeded) (bits/sec)",    OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL); 
	retrans_handle					= op_stat_reg ("Wireless Lan.Retransmission Attempts (packets)",                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL); 
	media_access_delay				= op_stat_reg ("Wireless Lan.Media Access Delay (sec)", 		                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);
	ete_delay_handle				= op_stat_reg ("Wireless Lan.Delay (sec)", 					 	                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);
	throughput_handle				= op_stat_reg ("Wireless Lan.Throughput (bits/sec)", 			                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);
	ap_conn_handle					= op_stat_reg ("Wireless Lan.AP Connectivity",		 			                     OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL);

	/* Register management traffic sent statistics only for APs.			*/
	if (ap_flag == OPC_BOOLINT_ENABLED)
		{
		mgmt_traffic_sent_handle		= op_stat_reg ("Wireless Lan.Management Traffic Sent (packets/sec)",             OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL); 
		mgmt_traffic_sent_handle_inbits	= op_stat_reg ("Wireless Lan.Management Traffic Sent (bits/sec)", 	             OPC_STAT_INDEX_NONE, OPC_STAT_LOCAL); 
		}

	/* Registering global statistics.										*/
	global_ete_delay_handle   = op_stat_reg ("Wireless LAN.Delay (sec)", 	  		                            OPC_STAT_INDEX_NONE, OPC_STAT_GLOBAL);
	global_load_handle 		  = op_stat_reg ("Wireless LAN.Load (bits/sec)", 		                            OPC_STAT_INDEX_NONE, OPC_STAT_GLOBAL);
	global_throughput_handle  = op_stat_reg ("Wireless LAN.Throughput (bits/sec)",                              OPC_STAT_INDEX_NONE, OPC_STAT_GLOBAL);
	global_buffer_drop_handle = op_stat_reg ("Wireless LAN.Data Dropped (Buffer Overflow) (bits/sec)",          OPC_STAT_INDEX_NONE, OPC_STAT_GLOBAL);
	global_retx_drop_handle   = op_stat_reg ("Wireless LAN.Data Dropped (Retry Threshold Exceeded) (bits/sec)", OPC_STAT_INDEX_NONE, OPC_STAT_GLOBAL);
	global_mac_delay_handle	  = op_stat_reg ("Wireless LAN.Media Access Delay (sec)",                           OPC_STAT_INDEX_NONE, OPC_STAT_GLOBAL);
	global_retrans_handle	  = op_stat_reg ("Wireless LAN.Retransmission Attempts (packets)",                  OPC_STAT_INDEX_NONE, OPC_STAT_GLOBAL);
	
	/* Register the "Network Load" dimensioned global statistic managed		*/
	/* using the oms_dim_stat_support package. Use the BSS ID as the key	*/
	/* for the dimension of the statistic reserved for our BSS.				*/
	sprintf (stat_info, "%d", bss_id);
	global_network_load_handle = Oms_Dim_Stat_Reg (my_objid, "Wireless LAN", "Network Load (bits/sec)", stat_info, OPC_STAT_GLOBAL);
	
	/* Initialize polling index. 								*/
	poll_index = -1;

	/* Initialize pcf retry count. 								*/
	pcf_retry_count = 0;

	/* Initialize pcf retry count. 								*/
	poll_fail_count = 0;

	/* Initialize pcf queue offset. 							*/
	pcf_queue_offset = 0;

	/* Initialize retry and back-off slot counts.				*/
	short_retry_count = 0;
	long_retry_count  = 0;
	backoff_slots     = BACKOFF_SLOTS_UNSET;

	/* Initialize the packet pointers that holds the last		*/
	/* transmitted packets to be used for retransmissions when	*/
	/* necessary.												*/
	wlan_transmit_frame_copy_ptr     = OPC_NIL;
    wlan_pcf_transmit_frame_copy_ptr = OPC_NIL;
	
	/* Initialize NAV duration and NAV reset event handle.		*/
	nav_duration  = 0;
	nav_reset_evh = invalid_evh;
	
	/* Initialize receiver idle timer. 							*/
	rcv_idle_time = -2.0 * difs_time;

	/* Similarly set the TBTT to negative beacon interval.		*/
	beacon_tx_time = -beacon_int;
	
	/* Initializing the sum of sizes of the packets in the		*/
	/* higher layer queue.										*/
	total_hlpk_size = 0;
	total_hlpk_num  = 0;
	
	/* Initialize the state variables related with the current	*/
	/* frame that is being handled.								*/
	packet_size_dcf  = 0;
	packet_size_pcf  = 0;
	receive_time_dcf = 0.0;
	receive_time_pcf = 0.0;
	
	/* Initializing frame response to send to none.				*/
	fresp_to_send = WlanC_None;

	/* Determines if the ap is controlling the medium. This		*/
	/* variable is used to determine when the NAV's can be		*/
	/* updates.													*/
	cfp_ap_medium_control = OPC_FALSE;

	/* Initializing expected frame type to none.				*/
	expected_frame_type = WlanC_None;

	/* Initialize the counter that will be used in assigning	*/
	/* sequence numbers to transmitted frames.					*/
	packet_seq_counter = 0;
	
	/* Set the variable that holds the current simulation time.	*/
	current_time = op_sim_time ();
		
	/* Set the flag indicating debugging mode.					*/
	debug_mode = op_sim_debug ();
	
	/* If running in debugging mode, initialize the variable	*/
	/* the name of the current state of the MAC.				*/
	if (debug_mode)
		strcpy (current_state_name, "INIT");
	
	/* Data arrived from higher layer is queued in the buffer. Pool memory is used for		*/
	/* allocating data structure for the higher layer packet and the random destination		*/
	/* for the packet. This structure is then inserted in the higher layer arrival queue.	*/
	hld_pmh = op_prg_pmo_define ("WLAN hld list elements", sizeof (WlanT_Hld_List_Elem), 32);

	/* Obtaining transmitter objid for accessing channel data rate attribute.	*/
	tx_objid = op_topo_assoc (my_objid, OPC_TOPO_ASSOC_OUT, OPC_OBJTYPE_RATX, 0);

	/* If no receiver is attach then generate error message and abort the simulation.	*/
	if (tx_objid == OPC_OBJID_INVALID)
		{
		wlan_error_print ("No transmitter attached to this MAC process", OPC_NIL, OPC_NIL);	
		}

	/* Obtaining number of channels available.									*/
	op_ima_obj_attr_get (tx_objid, "channel", &chann_objid);
	num_chann = op_topo_child_count (chann_objid, OPC_OBJTYPE_RATXCH);
	
	/* Check for error conditions. The transmitter is expected to have a single	*/
	/* channel.																	*/
	if (num_chann > 1)
		wlan_error_print ("The transmitter of the surrounding node has too many channels. This MAC",
						  "is implemented to use a single channel for all supported data rates.",
						  "Possibly, the new MAC process model is deployed in an old node model.");
	else if (num_chann == 0)
		wlan_error_print ("No channel is available for transmission.", OPC_NIL, OPC_NIL);
		
	/* Set the transmitter's transmission power.								*/	
	txch_objid = op_topo_child (chann_objid, OPC_OBJTYPE_RATXCH, 0);
	op_ima_obj_attr_set (txch_objid, "power", tx_power);

	/* Free the transmitter channel state information set at the rxgroup		*/
	/* pipeline stage.															*/
	temp_ptr = (void *) op_ima_obj_state_get (txch_objid);
	op_prg_mem_free (temp_ptr);	
	
	/* Reset the channel state. */
	op_ima_obj_state_set (txch_objid, OPC_NIL);

	/* Obtaining receiver's objid for accessing channel data rate attribute.	*/
	rx_objid = op_topo_assoc (my_objid, OPC_TOPO_ASSOC_IN, OPC_OBJTYPE_RARX, 0);

	/* If no receiver is attach then generate error message and abort the		*/
	/* simulation.																*/
	if (rx_objid == OPC_OBJID_INVALID)
		{
		wlan_error_print ("No receiver attached to this MAC process", OPC_NIL, OPC_NIL);	
		}

	/* Disable the apptracking capability for the receiver						*/
	apptrack_rx_processing_disable (rx_objid);
	
	/* Obtaining number of channels available.									*/
	op_ima_obj_attr_get (rx_objid, "channel", &chann_objid);
	num_chann = op_topo_child_count (chann_objid, OPC_OBJTYPE_RARXCH);
	
	/* Check for error conditions. The receiver is expected to have a single	*/
	/* channel.																	*/
	if (num_chann > 1)
		wlan_error_print ("The receiver of the surrounding node has too many channels. This MAC",
						  "is implemented to use a single channel for all supported data rates.",
						  "Possibly, the new MAC process model is deployed in an old node model.");
	else if (num_chann == 0)
		wlan_error_print ("No channel is available for reception.", OPC_NIL, OPC_NIL);

	/* Free the receiver channel state information set at the rxgroup stage.	*/
	rxch_objid = op_topo_child (chann_objid, OPC_OBJTYPE_RARXCH, 0);
	temp_ptr = (void *) op_ima_obj_state_get (rxch_objid);
	op_prg_mem_free (temp_ptr);
	
	/* Reset the channel state. */
	op_ima_obj_state_set (rxch_objid, OPC_NIL);
		
	/* Initialize the roaming related information.								*/
	roam_state_ptr = (WlanT_Roam_State_Info *) op_prg_mem_alloc (sizeof (WlanT_Roam_State_Info));
	roam_state_ptr->ap_reliability = 1.0;
	roam_state_ptr->scan_mode      = OPC_FALSE;
	roam_state_ptr->current_bss_id = bss_id;
	roam_state_ptr->scan_type      = WlanC_Scan_Type_Beacon;

	/* Initially, set roaming based on the attribute. In the next state, if it	*/
	/* is determined that this is an ad-hoc network or if this STA is in the	*/
	/* PCF polling-list of the AP, then the roaming will be disabled.			*/
	op_ima_obj_attr_get (params_attr_objid, "Roaming Capability", &roaming_cap_flag);
	roam_state_ptr->enable_roaming = (roaming_cap_flag == OPC_BOOLINT_ENABLED) ? OPC_TRUE : OPC_FALSE;

	/* Initialize the receiver channel state information.						*/
	rx_state_info_ptr = (WlanT_Rx_State_Info *) op_prg_mem_alloc (sizeof (WlanT_Rx_State_Info));
	rx_state_info_ptr->state_info_id 	        = WLANC_RXCH_STATE_ID;
	rx_state_info_ptr->rx_power_thresh          = rx_power_threshold;
	rx_state_info_ptr->rx_end_time              = 0.0;
	rx_state_info_ptr->roaming_info_ptr         = roam_state_ptr;
	rx_state_info_ptr->phy_tech			        = phy_type;
	rx_state_info_ptr->routed_bgutil_state_ptr	= OPC_NIL;
	rx_state_info_ptr->port_name_ptr       	    = OPC_NIL;
	rx_state_info_ptr->node_name_ptr     	    = OPC_NIL;
   	rx_state_info_ptr->data_rate            	= data_tx_rate;
   	rx_state_info_ptr->congestion_area          = OPC_FALSE;
	rx_state_info_ptr->wlan_pk_rx_end_time		= 0.0;
	rx_state_info_ptr->jammer_rx_end_time		= 0.0;
	rx_state_info_ptr->busy_due_to_jammer		= OPC_FALSE;
	
	/* Find out the objid of the packet stream from the receiver into the MAC	*/
	/* module, which is also stored in receiver channel state information.		*/
	rx_state_info_ptr->mac_strm_objid   = op_topo_assoc (rx_objid, OPC_TOPO_ASSOC_OUT, OPC_OBJTYPE_STRM, 0);
	
	/* Set the new "state" information of the receiver channel.					*/
	op_ima_obj_state_set (rxch_objid, rx_state_info_ptr);
	
	/* Also overwrite the high threshold trigger attribute values of the		*/
	/* statwires that come into the MAC from the radio receiver by using the	*/
	/* reception power threshold. First determine the total count of incoming	*/
	/* statwires.																*/
	num_statwires = op_topo_assoc_count (my_objid, OPC_TOPO_ASSOC_IN, OPC_OBJTYPE_STATWIRE);
	for (i = 0; i < num_statwires; i++)
		{
		/* Access the next statwire. Skip it if it is coming from the			*/
		/* transmitter.															*/
		statwire_objid = op_topo_assoc (my_objid, OPC_TOPO_ASSOC_IN, OPC_OBJTYPE_STATWIRE, i);
		op_ima_obj_attr_get (statwire_objid, "high threshold trigger", &threshold);
		
		/* If the trigger is not disabled then the statwire is from the			*/
		/* receiver. Overwrite the attribute value unless they are already same.*/
		if (threshold != OPC_BOOLDBL_DISABLED && threshold != rx_power_threshold)
			op_ima_obj_attr_set (statwire_objid, "high threshold trigger", rx_power_threshold);			
		}
			
	/* Create an ICI to be used during the communication with LLC.				*/
	llc_iciptr = op_ici_create ("wlan_mac_ind");
	if (llc_iciptr == OPC_NIL)
		{
		wlan_error_print ("Unable to create ICI for communication with LLC.", OPC_NIL, OPC_NIL);
		}

	/* Initialize the variable which keeps track of number of PCF enabled nodes	*/
	/* in the network 															*/	
	pcf_network = 0;
	
	/* Create the mutex that will be used to serialize calling of prg_mapping	*/
	/* functions, which read/write global model related mapping information,	*/
	/* under multi-threaded execution with multiple CPUs.						*/
	mapping_info_mutex = op_prg_mt_mutex_create (OPC_MT_MUTEX_READER_WRITER, 0, "WLAN Mapping Info Mutex");

	/* Unless already done by another WLAN MAC process model, initialize the	*/
	/* global variables that hold the indices of the fields of the WLAN packet	*/
	/* formats.																	*/
	if (WLANC_DATA_TYPE_FD == OPC_FIELD_INDEX_INVALID)
		wlan_pk_field_indices_initialize ();
	
	FOUT;
	}

static void
wlan_transceiver_channel_init (void)
	{
	Objid		mac_params_comp_attr_objid;
	Objid		params_attr_objid;
	Objid		chann_params_comp_attr_objid;
	Objid		subchann_params_attr_objid;	
	int			bss_index;
	int			found_channel_num;
	double		bandwidth_mhz;
	double		frequency;
	char		err_msg1 [256];

	/** This function determines the WLAN channel that the surrounding	**/
	/** node will use and configures the transceiver with the			**/
	/** corresponding minimum frequency and bandwidth values.			**/
	FIN (wlan_transceiver_channel_init (void));
	
	/* Get a handle to node's WLAN parameters.							*/
	op_ima_obj_attr_get (my_objid, "Wireless LAN Parameters", &mac_params_comp_attr_objid);
    params_attr_objid = op_topo_child (mac_params_comp_attr_objid, OPC_OBJTYPE_GENERIC, 0);

	/* Get the provided channel configuration.							*/
	op_ima_obj_attr_get (params_attr_objid, "Channel Settings", &chann_params_comp_attr_objid);
	subchann_params_attr_objid = op_topo_child (chann_params_comp_attr_objid, OPC_OBJTYPE_GENERIC, 0);
	op_ima_obj_attr_get (subchann_params_attr_objid, "Bandwidth", &bandwidth_mhz);	
	op_ima_obj_attr_get (subchann_params_attr_objid, "Min Frequency", &frequency);	

	/* Check whether the bandwidth is specified using a special value.	*/
	if (bandwidth_mhz == WLANC_PHY_BASED_BW_USED)
		{
		/* Set the bandwidth to the standard value of used physical		*/
		/* layer technology.											*/
		bandwidth_mhz = (phy_char_flag == WlanC_OFDM_11a) ? WLANC_11a_CHNL_BANDWIDTH : WLANC_11b_CHNL_BANDWIDTH;
		}
	
	/* Determine the channel number we are going to use.				*/ 
	if (frequency == WLANC_BSS_BASED_FREQ_USED)
		{
		/* The channel will be selected based on node's BSS ID. Check	*/
		/* whether the BSS IDs are auto-assigned or not.				*/
		if (bss_id_type == WlanC_Entire_Subnet)
			{
			/* BSS IDs are auto assigned and their are set to node's	*/
			/* subnet ID. Use BSS index for channel selection.			*/
			bss_index = wlan_bss_id_list_manage (bss_id, phy_type, "get_index");
			}
		else
			{
			/* BSS IDs are explicitly assigned. Use the BSS ID as the	*/
			/* BSS index.												*/
			bss_index = bss_id;
			}
		
		/* Assign channels from 1 through 11 (North America) with a gap */
		/* of 5 channels in between. The sequence will look like 1, 6,  */
		/* 11, 5, 10, 4, 9,... The channel will be picked as a function */
		/* BSS ID, that is mapped to a BSS index. Note that only		*/
		/* channels that are 5 channels apart are non-overlapping.		*/
		/* Assume that BSS indices run serially: 0, 1, 2, ... 			*/
		/* WLANC_CHANNEL_COUNT = 11, and								*/
		/* WLANC_CH_STEP_FOR_NO_OVERLAP = ceil (22.0/5.0) = 5			*/
		/*																*/
		/* Follow the same logic in case of 802.11a operation, where	*/
		/* WLANC_CHANNEL_COUNT = 12, and								*/
		/* WLANC_CH_STEP_FOR_NO_OVERLAP = ceil (20.0/20.0) = 1 (i.e.,	*/
		/* in 802.11a, the adjacent WLAN channels don't overlap.)		*/
		channel_num = (WLANC_CH_STEP_FOR_NO_OVERLAP * bss_index) % channel_count + 1;

		/* Get the minimum frequency that corresponds to the selected	*/
		/* channel.														*/
		frequency = wlan_min_freq_for_chan (channel_num, phy_type);
		}
	else
		{
		/* The frequency band that will be used by this node is			*/
		/* explicitly specified. Validate it and determine its channel	*/
		/* number. 														*/
		
		/* In 802.11/11b/11g networks, for a standard, acceptable		*/
		/* channel, the bandwidth has to be 22 MHz, and the minimum		*/
		/* frequency between 2401 and 2451 MHz, inclusive, with			*/
		/* intervals of 5 MHz (i.e. the roaming feature is currently	*/
		/* supported only within first 11 channels. These numbers refer	*/
		/* to the values of the constants used below as specified in	*/
		/* the standard).												*/
		if (phy_char_flag != WlanC_OFDM_11a  && bandwidth_mhz == channel_bandwidth && frequency >= first_chan_min_freq && 
			frequency == ceil (frequency)    && (frequency - first_chan_min_freq) / channel_spacing < channel_count && 
			(int) (frequency - first_chan_min_freq) % (int) channel_spacing == 0)
			{
			/* The radio band has a valid configuration. Compute its	*/
			/* channel number.											*/
			channel_num = (int) ((frequency - first_chan_min_freq) / channel_spacing + 1);
			}
		
		/* In case of 802.11a, search the array of the standard channel	*/
		/* to check the validity of the specified channel band.			*/
		else if (phy_char_flag == WlanC_OFDM_11a && bandwidth_mhz == channel_bandwidth && 
				 wlan_11a_channel_is_regular (frequency, &found_channel_num))
			{
			channel_num = found_channel_num;
			}
		
		/* Invalid (irregular) channel bands are acceptable if the node	*/
		/* is not expected to roam.										*/
		else if (!roam_state_ptr->enable_roaming)
			{
			/* When the roaming is disabled, the value of the state		*/
			/* variable channel_num is not used by regular MACs. For	*/
			/* APs, this means that the APs with irregular channel		*/
			/* bandwidths cannot support roaming WLAN MACs, since they	*/
			/* will not be able to discover such an AP during scanning.	*/
			channel_num = 0;
			}
			
		else
			{
			/* Terminate the simulation with an error message.			*/
			sprintf (err_msg1, "bandwidth must be %.1f MHz for 802.11a networks, and %.1f MHz for 802.11/11b/11g networks.",
					 WLANC_11a_CHNL_BANDWIDTH, WLANC_11b_CHNL_BANDWIDTH);
			wlan_error_print ("Cannot support WLAN roaming with configured radio channel settings. Roaming is supported", 
							  "only for the operation channels specified in 802.11a and 802.11/11b/11g standards. Channel", err_msg1);
			}
		}
	
	/* Configure the transmitter channel based on selected/assigned		*/
	/* frequency band.													*/
	op_ima_obj_attr_set (txch_objid, "bandwidth", bandwidth_mhz * 1000.0);
	op_ima_obj_attr_set (txch_objid, "min frequency", frequency);
	
	/* Similarly configure the receiver channel.						*/
	op_ima_obj_attr_set (rxch_objid, "bandwidth", bandwidth_mhz * 1000.0);
	op_ima_obj_attr_set (rxch_objid, "min frequency", frequency);
			
	FOUT;
	}

static void
wlan_higher_layer_data_arrival (void)
	{
	Packet*					hld_pkptr;
	OpT_Packet_Size			data_size, frag_size;
	OpT_Int64				dest_addr; 
	int						protocol_type;
	Ici*					ici_ptr;
	Boolean					polling;

	/** Start the processing of the higher layer data packet and	**/
	/** queue it if the packet is accepted and there is sufficient	**/
	/** space in the higher layer data buffer.						**/
	FIN (wlan_higher_layer_data_arrival (void));

	/* Get packet from the incoming stream from higher layer and	*/
	/* obtain the packet size 										*/
	hld_pkptr = op_pk_get (instrm_from_mac_if);	
	
	/* If we are in a bridge/switch node, then we don't accept any	*/
	/* higher layer packet unless we are AP enabled.				*/
	if ((wlan_flags->bridge_flag == OPC_TRUE) && (ap_flag == OPC_BOOLINT_DISABLED))
		{
		op_pk_destroy (hld_pkptr);
		FOUT;
		}

	/* Read ICI parameters at the stream interrupt.					*/
	ici_ptr = op_intrpt_ici ();

	/* Retrieve destination address from the ICI set by the			*/
	/* interface layer.												*/
	if (ici_ptr == OPC_NIL || op_ici_attr_get_int64 (ici_ptr, "dest_addr", &dest_addr) == OPC_COMPCODE_FAILURE)
		{
		/* Generate error message.									*/
		wlan_error_print ("Destination address is not valid.", OPC_NIL, OPC_NIL);
		}

	/* Get the protocol information of higher layer data from ICI.	*/
	op_ici_attr_get (ici_ptr, "protocol_type", &protocol_type);
		
	/* Check for an AP bridge that whether the destined stations	*/
	/* exist in the BSS or not. If not then no need to broadcast	*/
	/* the packet.													*/
	if (wlan_flags->bridge_flag == OPC_TRUE && ap_flag == OPC_BOOLINT_ENABLED)
		{
		/* If the destination station doesn't exist in the BSS then	*/
		/* no need to broadcast the packet.							*/
		if (dest_addr >= 0 && prg_bin_hash_table_item_get (peer_info_hash_tbl, (void *) &(dest_addr)) == PRGC_NIL)
			{
			op_pk_destroy (hld_pkptr);
			
			FOUT;	
			}
		}

	/* Get the size of the packet arrived from higher layer.		*/
	data_size = op_pk_total_size_get (hld_pkptr);		
	
	/* Update the local and global load statistics.					*/
	op_stat_write (packet_load_handle, 1.0);
	op_stat_write (packet_load_handle, 0.0);
	op_stat_write (bits_load_handle,   (double) data_size);
    op_stat_write (bits_load_handle,   0.0);
	op_stat_write (global_load_handle, (double) data_size);
    op_stat_write (global_load_handle, 0.0);

	/* Determine the data size of the MPDUs of the MSDU.			*/
	if ((data_size > frag_threshold) && (frag_threshold != -1))
		frag_size = frag_threshold;
	else
		frag_size = data_size;
	
	/* Destroy packet if it is more than max msdu length or its		*/
	/* size zero. Also, if the size of the higher layer queue  		*/
	/* will exceed its maximum after the insertion of this packet, 	*/
	/* then discard the arrived packet. 							*/
	/* The higher layer is responsible for the retransmission of 	*/
	/* this packet.													*/ 
	if ((data_size > WLANC_MAXMSDU_LENGTH && accept_large_packets == OPC_BOOLINT_DISABLED) ||
		frag_size > WLANC_MAXMSDU_LENGTH || data_size == 0 || total_hlpk_size + data_size > hld_max_size)
		{
		/* Drop the higher layer packet.							*/
		wlan_hl_packet_drop (hld_pkptr, data_size);
		
		FOUT; 
		}
		
	/* Stamp the packet with the current time. This information		*/
	/* will remain unchanged even if the packet is copied for		*/
	/* retransmissions, and	eventually it will be used by the		*/
	/* destination MAC to compute the end-to-end delay.				*/
	op_pk_stamp (hld_pkptr);
	
	/* The queuing of the packets in the AP is based on the node	*/
	/* type	(DCF/PCF) of the destination. If the destination node	*/
	/* is a PCF	node, then the packets are queued into the			*/
	/* cfpd_list_ptr and transmitted only during the PCF period.	*/
	/* Likewise packets for a DCF destination node will be inserted	*/
	/* into the hld_list_ptr and transmitted only during DCF.		*/
	/* Packets with broadcast address are transmitted during DCF.	*/
	polling = wlan_poll_list_member_find (dest_addr);
	
	/* Maintaining total size and count of the packets in the		*/
	/* higher layer queue(s).										*/
	total_hlpk_num++;
	total_hlpk_size = total_hlpk_size + data_size;

	/* Tag the packet as it enters the queue for application delay	*/
	/* tracking.													*/
	apptrack_tx_enqueue_flag (hld_pkptr);

	/* Insert the arrived packet in higher layer queue.				*/	
	wlan_hlpk_enqueue (hld_pkptr, dest_addr, my_address, protocol_type, data_size, polling);

	FOUT;
	}


static void
wlan_hl_packet_drop (Packet* hld_pkptr, OpT_Packet_Size data_size)
	{
	int		large_packet_bit = 0x1;
	int		full_buffer_bit  = 0x2;

	/** This function drops the higher layer packets or packets **/
	/** received by the AP and need to be forwarded within the	**/
	/** BSS that can be accepted because of full buffer or		**/
	/** large size of the packet. It also writes an appropriate	**/
	/** log message to report the rejection unless the same log	**/
	/** message is already written before.						**/
	FIN (wlan_hl_packet_drop (hld_pkptr, data_size));
	
	/* Write an appropriate simulation log message unless the	*/
	/* same message is written before.							*/
	if (drop_pkt_entry_log_flag < full_buffer_bit + large_packet_bit)
		{
		if (total_hlpk_size + data_size > hld_max_size && !(drop_pkt_entry_log_flag & full_buffer_bit))
			{
			/* Writing log message for dropped packets.			*/
			op_prg_log_entry_write (drop_pkt_log_handle, 
				"SYMPTOM(S):\n"
				" Wireless LAN MAC layer discarded some packets due to\n "
			    " insufficient buffer capacity. \n"
				"\n"
			    " This may lead to: \n"
  			    " - application data loss.\n"
			    " - higher layer packet retransmission.\n"
			    "\n"
			    " REMEDIAL ACTION(S): \n"
			    " 1. Reduce network load. \n"
			    " 2. Use a higher wireless LAN data rate. \n"
			    " 3. Increase buffer capacity\n");
			drop_pkt_entry_log_flag += full_buffer_bit;
			}
			
		else if (total_hlpk_size + data_size <= hld_max_size && data_size > 0 && !(drop_pkt_entry_log_flag & large_packet_bit))
			{
			/* Writing log message for dropped packets due to	*/
			/* packet size.										*/
			op_prg_log_entry_write (drop_pkt_log_handle, 
				"SYMPTOM(S):\n"
			    " Wireless LAN MAC layer discarded some packets due to \n"
			    " their large sizes. This is an expected protocol \n"
				" behavior. \n"	
				"\n"
			    " This may lead to: \n"
  			    " - application data loss.\n"
			    " - higher layer packet retransmission.\n"
			    "\n"
			    " SUGGESTION(S): \n"
			    " 1. Set the higher layer packet size to \n"
				"    be smaller than max MSDU size (2304 bytes). \n"
			    " 2. Enable fragmentation threshold and large packet \n"
				"    processing. \n");
			drop_pkt_entry_log_flag += large_packet_bit;
			}
		}

	/* Destroy the dropped packet.								*/
	op_pk_destroy (hld_pkptr);
		
	/* Update the local and global statistics that record the	*/
	/* dropped data traffic due to full buffer or large packet	*/
	/* size.													*/
	op_stat_write (buffer_drop_pkts_handle, 1.0);
	op_stat_write (buffer_drop_pkts_handle, 0.0);
	op_stat_write (buffer_drop_bits_handle, (double) (data_size));
	op_stat_write (buffer_drop_bits_handle, 0.0);
	op_stat_write (global_buffer_drop_handle, (double) (data_size));
	op_stat_write (global_buffer_drop_handle, 0.0);
	
	FOUT;
	}


static void
wlan_hlpk_enqueue (Packet* hld_pkptr, OpT_Int64 dest_addr, OpT_Int64 orig_src_addr, int protocol_type, OpT_Packet_Size data_size, Boolean polling)
	{
	char					msg_string [120];
	char					msg_string1 [120];
	WlanT_Hld_List_Elem*	hld_ptr;
	
	/* Enqueuing data packet for transmission.	*/
	FIN (wlan_hlpk_enqueue (hld_pkptr, dest_addr, orig_src_addr, protocol_type, data_size, polling));

	/* Allocating pool memory to the higher layer data structure type. */	
	hld_ptr = (WlanT_Hld_List_Elem *) op_prg_pmo_alloc (hld_pmh);

	/* Generate error message and abort simulation if no memory left for data received from higher layer.	*/
	if (hld_ptr == OPC_NIL)
		{
		wlan_error_print ("No more memory left to assign for data received from higher layer", OPC_NIL, OPC_NIL);
		}

	/* Updating higher layer data structure fields.	*/
	hld_ptr->time_rcvd           = current_time;
	hld_ptr->destination_address = dest_addr;
	hld_ptr->origination_address = orig_src_addr;
	hld_ptr->protocol_type		 = protocol_type;
	hld_ptr->pkptr               = hld_pkptr;

	/* Check for PCF terminal and also if this station has been polled.	*/
	if (polling == OPC_TRUE)
		{
		/* Insert the packet sorted in the order of the MAC addresses.	*/
		op_prg_list_insert_sorted (cfpd_list_ptr, hld_ptr, wlan_hld_list_elem_add_comp);	
		}
	else
		{
		/* Insert a packet to the list.*/
		op_prg_list_insert (hld_list_ptr, hld_ptr, OPC_LISTPOS_TAIL);	
		
		/* Enable the flag indicating that there is a data frame to		*/
		/* transmit.													*/
		wlan_flags->data_frame_to_send = OPC_TRUE;
		}

	/* Update the queue size statistic.									*/
	op_stat_write (hl_packets_rcvd, (double) total_hlpk_num);
	
	/* Queued packets are also recorded under the network load			*/
	/* statistic.														*/
	Oms_Dim_Stat_Write (global_network_load_handle, (double) data_size);
	Oms_Dim_Stat_Write (global_network_load_handle, 0.0);
	
	/* Printing out information to ODB.									*/
	if (wlan_trace_active == OPC_TRUE)
		{
		sprintf (msg_string, "Just arrived outbound Data packet id " OPC_PACKET_ID_FMT ".", op_pk_id (hld_ptr->pkptr));
		sprintf	(msg_string1, "The outbound Data queue size is %d.", total_hlpk_num); 	
		op_prg_odb_print_major (msg_string, msg_string1, OPC_NIL);
		}

	FOUT;
	}

static void 
wlan_frame_transmit ()
	{
	char							msg_string  [120];
	char							msg_string1 [120];
	WlanT_Hld_List_Elem*			hld_ptr;
	const WlanT_Data_Header_Fields*	retx_header_ptr;
	int 							list_high_index;
	int								list_low_index;
	WlanT_Mac_Frame_Type			type;
	Boolean							pcf_frag_buf_empty;
	
	/** Main procedure to invoke function for preparing and  **/
	/** transmitting the appropriate frames.			     **/
	FIN (wlan_frame_transmit());

	/* If the receiver is busy due to a wlan packet reception, then set the bad_packet_dropped flag	*/
	/* So that if another reception starts during our transmission before this current reception	*/
	/* ends, we don't set the rcvd_bad_packet flag incorrectly to drop next delivered packet, since	*/
	/* there won't be such a packet																	*/
	if ((wlan_flags->receiver_busy) && (rx_state_info_ptr->wlan_pk_rx_end_time >= current_time))
		wlan_flags->bad_packet_dropped = OPC_TRUE;
	
	/* Check if PCF is currently active and if time to transmit */
	/* the CFP end frame. If so check if more fragments have 	*/
	/* to be transmitted. If none then, prepare to send the 	*/
	/* cfp_end frame to indicate the end of the CFP period		*/
	if ((wlan_flags->pcf_active == OPC_TRUE) && 
		((wlan_flags->tx_beacon == OPC_FALSE) || 
		 (wlan_flags->tx_cf_end == OPC_TRUE)  ||
		 (op_sar_buf_size (pcf_frag_buffer_ptr) != 0)))
		{
		/* Store the size of the PCF fragmentation buffer in a	*/
		/* local variable for quick access.						*/
		pcf_frag_buf_empty = (op_sar_buf_size (pcf_frag_buffer_ptr) == 0) ? OPC_TRUE : OPC_FALSE;
		
		/* Check if the transmission of the cf end frame has been */
		/* enabled. If so, make sure there are no more fragments  */
		/* pending and the PCF fragmentation buffer is empty      */
		if ((wlan_flags->tx_cf_end == OPC_TRUE) && (wlan_flags->more_frag == OPC_FALSE) && pcf_frag_buf_empty)
			{
			/* If the AP needs to ACK to a previously received       */
			/* frame send a CF_end_Ack frame, if not transmit CF end */
			if (fresp_to_send == WlanC_Ack) 
				wlan_prepare_frame_to_send (WlanC_Cf_End_A);
			else  
				wlan_prepare_frame_to_send (WlanC_Cf_End);
						
			FOUT;
			}

		/* Allocating pool memory to the higher layer data structure type. */	
		hld_ptr = (WlanT_Hld_List_Elem *) op_prg_pmo_alloc (hld_pmh);
	
		/* Generate error message and abort simulation if no memory left for data received from higher layer.	*/
		if (hld_ptr == OPC_NIL)
			{
			wlan_error_print ("No more memory left to assign for data received from higher layer", OPC_NIL, OPC_NIL);
			}
		
		/* Set up dummy element to see if any more data for station currently being polled */
		if (poll_index > -1)
			hld_ptr->destination_address = polling_list [poll_index];
		else 
			hld_ptr->destination_address = -1;
		
		/* Set search bound for pcf higher layer data queue */
		list_high_index = op_prg_list_size (cfpd_list_ptr);
		list_low_index = 0;

		/* If a poll fail count reached the max poll fail count or		 */
		/* the previous poll was successful and no more data from this   */
		/* station and last data tx was successful and no more           */
		/* fragments exist and no more data exist in the hlk queue       */
		/* for this station then next station will start transmission	 */
		if ((poll_fail_count > max_poll_fails) ||
			(((poll_fail_count == 0) && 
			 (wlan_flags->more_data == OPC_FALSE) &&
			 (wlan_flags->more_frag == OPC_FALSE)) &&
			((pcf_retry_count == 0) && pcf_frag_buf_empty &&
			(op_prg_list_elem_find (cfpd_list_ptr, wlan_hld_list_elem_add_comp, hld_ptr, 
			&list_low_index, &list_high_index) == OPC_NIL))))
			{
			/* Increment polling index to next user.					*/
			poll_index++;
			
			/* Check whether the poll reached the specified limit.		*/
			if (poll_fail_count > max_poll_fails)
				{
				/* Reset the relevant flags.							*/
				wlan_flags->pcf_side_traf = OPC_FALSE;
				wlan_flags->active_poll   = OPC_FALSE;
				wlan_flags->more_data	  = OPC_FALSE;
			
				/* Set the retry count to the retry limit to drop the	*/
				/* packet.												*/
				pcf_retry_count = long_retry_limit;
				
				/* Drop the packet. The function will also reset the	*/
				/* counter for failed polls.							*/
				wlan_pcf_frame_discard ();
				}
			}
		
		/* If we finished polling all the pollable STAs in the list but	*/
		/* still have some contention free frames to send, then restart	*/
		/* polling the pollable STAs since we still have some CFP time	*/
		/* to go.														*/
		if (poll_index == poll_list_size)
			{
			if (op_prg_list_size (cfpd_list_ptr) > 0)
				{
				/* Restart the polling.									*/
				poll_index       = 0;
				pcf_queue_offset = 0;
				}
			else
				{
				/* End the CFP prematurely since we have no stations to	*/
				/* poll and no CF frames to send. Also send an ACK if	*/
				/* necessary.											*/
				if (fresp_to_send == WlanC_Ack) 
					wlan_prepare_frame_to_send (WlanC_Cf_End_A);
				else 
					wlan_prepare_frame_to_send (WlanC_Cf_End);
				
				/* Destroy the dummy higher layer data entry used for	*/
				/* searching.											*/
				op_prg_mem_free (hld_ptr);
			
				FOUT;			
				}
			}

		/* Determine our data rate for our next poll. This is necessary	*/
		/* if we are an 11g AP serving both 11g and 11b STAs.			*/
		if (phy_type == WlanC_11g_PHY && wlan_flags->non_erp_present && data_tx_rate > 5500000.0 && data_tx_rate != 11000000.0)
			{
			/* First check whether we are polling the same STA.			*/
			if (hld_ptr->destination_address == polling_list [poll_index])
				{
				/* Same STA. Don't change the data rate unless			*/
				/* previously we polled this 11g STA with low data rate	*/
				/* because that poll had a piggybacked ACK for an 11b	*/
				/* STA.													*/
				if (wlan_flags->pcf_lowered_drate)
					{
					/* Go back to the regular rate and reset the flag.	*/
					operational_speed = data_tx_rate;
					wlan_flags->pcf_lowered_drate = OPC_FALSE;
					}
				}
			else
				{
				/* Check whether the new STA 11g enabled.				*/
				if (wlan_dest_is_11g_enabled (polling_list [poll_index]))
					{
					/* Use the regular rate for the new 11g enabled STA	*/
					/* unless we also need to ACK an 11b STA.			*/
					if (operational_speed != data_tx_rate)
						{
						if (fresp_to_send != WlanC_Ack || wlan_flags->pcf_lowered_drate == OPC_TRUE)
							{
							/* Adjust the rate and reset the flag.		*/
							operational_speed = data_tx_rate;
							wlan_flags->pcf_lowered_drate = OPC_FALSE;
							}
						else
							{
							/* We can't increase the rate for the		*/
							/* current transmission because we also		*/
							/* need to ACK an 11b STA. Set the flag so	*/
							/* that we adjust the data rate before		*/
							/* the next poll.							*/
							wlan_flags->pcf_lowered_drate = OPC_TRUE;
							}
						}
					}
				else if (operational_speed == data_tx_rate)
					{
					/* The new STA is a non-ERP STA (not 11g enabled).	*/
					/* Lower our data rate so that it can decode our	*/
					/* transmission. Pick the highest 11b data rate		*/
					/* that is lower than our regular 11g data rate.	*/
					if (data_tx_rate > 11000000.0)
						operational_speed = 11000000.0;
					else if (data_tx_rate > 5500000.0)
						operational_speed = 5500000.0;
					}
				else
					/* Go on using lowered data rate since the new STA	*/
					/* is not 11g enabled. Reset the flag, which may be	*/
					/* set.												*/
					wlan_flags->pcf_lowered_drate = OPC_FALSE;
				}
			}
		
		/* Re init dummy element for new poll index.					*/
		hld_ptr->destination_address = polling_list [poll_index];

		/* Set the destination address.									*/				
		pcf_destination_addr = hld_ptr->destination_address;
		
		/* First check if this is a retry.								*/
		if (pcf_retry_count != 0)
			{
			/* Destroy the dummy higher layer data entry used for		*/
			/* searching.												*/
			op_prg_mem_free (hld_ptr);
			
			/* Set type to last frame type.								*/
			op_pk_fd_get_int32 (wlan_pcf_transmit_frame_copy_ptr, WLANC_DATA_TYPE_FD, (int *) &type);
			
			/* Retrieve the destination information from the frame.		*/
			op_pk_fd_access_read_only_ptr (wlan_pcf_transmit_frame_copy_ptr, WLANC_DATA_HEADER_FD, (const void **) &retx_header_ptr);
			pcf_destination_addr = (retx_header_ptr->tods == 1) ? retx_header_ptr->address3 : retx_header_ptr->address1;
			
			/* Check the ACK status for retransmission.					*/
			if (fresp_to_send != WlanC_Ack && (type == WlanC_Data_A_P || type == WlanC_Cf_A_P))
				{
				/* The previous message was sent with an ACK, which		*/
				/* needs to be removed from this retransmission.		*/
				type = (type == WlanC_Data_A_P) ? WlanC_Data_Poll : WlanC_Cf_Poll;
				}
			
			/* Perform the retransmission.								*/
			wlan_prepare_frame_to_send (type);
			
			FOUT;
			}
		
		/* Check if fragmentation buffer is empty and if there is any data to send 	*/
	   	/* to this station.  If no data,  send  ack / poll as needed.		 		*/ 
		else if (pcf_frag_buf_empty && 
				(op_prg_list_elem_find (cfpd_list_ptr, wlan_hld_list_elem_add_comp, hld_ptr, &list_low_index, &list_high_index) == OPC_NIL))
			{
			/* Set active poll flag since poll will be transmitted*/
			wlan_flags->active_poll = OPC_TRUE;

			/* If the AP has a pending ACK to transmit, send Ack-CF poll frame. */
			/* If no pending ACK for this station transmit the poll frame		*/
			if (fresp_to_send == WlanC_Ack) 
				{
				wlan_prepare_frame_to_send (WlanC_Cf_A_P);
				}
			else
				{
				wlan_prepare_frame_to_send (WlanC_Cf_Poll);
				}
		
			/* Destroy the dummy higher layer data entry	*/
			/* used for searching.							*/
			op_prg_mem_free (hld_ptr);
			
			FOUT;
			}

		/* If we've come this far, there must be data for this user.		*/
		/* If the fragmentation buffer is empty, get a new packet and 	 	*/
		/* setup fragmentation buffer. Tx of frame is queued outside		*/
		/* this else if.													*/
		else if (pcf_frag_buf_empty) 
			{
			/* First destroy the dummy higher layer data entry used for		*/
			/* searching.													*/
			op_prg_mem_free (hld_ptr);
			
			do
				{
				/* Get next packet for transmission from the higher layer queue.*/
				hld_ptr = (WlanT_Hld_List_Elem*) op_prg_list_access (cfpd_list_ptr, pcf_queue_offset);

				/* Make sure destination address matches polling address.		*/
				if (hld_ptr->destination_address != polling_list [poll_index])
					{
					/* A packet must have been inserted into the queue by the 	*/
					/* upper layers after I started polling for a lower 		*/
					/* address.  Increment an offset to track packets at the 	*/
					/* head of the queue that have missed their opportunity 	*/
					/* to transmit this CFP.  Restore the packet to the 		*/
					/* point where it was stored, and get the next packet for 	*/
					/* transmission.											*/
					pcf_queue_offset++;
				
					if (pcf_queue_offset > list_high_index)
						{
						wlan_error_print ("Polling routine error. \n Destination not on list.", OPC_NIL, OPC_NIL);
						}
					}
				} 
			while (hld_ptr->destination_address != polling_list [poll_index] );
			
			/* Remove packet from higher layer queue. 						*/	
			hld_ptr = (WlanT_Hld_List_Elem *) op_prg_list_remove (cfpd_list_ptr, pcf_queue_offset);

//自添加
			//$$$$$$$$$$$$$$$$$$ DSR $$$$$$$$$$$$$$$$$$$$$$$$
			op_pk_nfd_get((Packet*)(hld_ptr->pkptr),"Type",&data_packet_type);
			if (data_packet_type == DATA_PACKET_TYPE)
				 {
				 op_pk_nfd_get((Packet*)(hld_ptr->pkptr),"NextHop",&data_packet_dest);
			  	 op_pk_nfd_get((Packet*)(hld_ptr->pkptr),"DEST",&data_packet_final_dest);
				 }
			else
				 {
				 data_packet_dest=-1;
				 data_packet_final_dest=-1;
				 }
			//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//自添加结束
			
			/* Setting address state variables.								*/				
			pcf_destination_addr = hld_ptr->destination_address;
			pcf_orig_source_addr = hld_ptr->origination_address;
			
			/* Determine packet size - required to determine fragmentation	*/
			packet_size_pcf = op_pk_total_size_get (hld_ptr->pkptr);
	
			/* Store the higher layer protocol information.					*/
			hl_protocol_pcf = hld_ptr->protocol_type;
			
			/* Assign the next sequence number to the packet and increment	*/
			/* the counter.													*/
			packet_seq_control = packet_seq_counter << WLANC_FRAG_NUM_SIZE;   
			packet_seq_counter = (packet_seq_counter + 1) % WLANC_SEQ_NUM_WRAP_VALUE;
						
			/* Packet needs to be fragmented if it is more than			*/
			/* fragmentation threshold, provided fragmentation is		*/
			/* enabled. Broadcast packets are not fragmented regardless	*/
			/* of their sizes.											*/
			if (frag_threshold != -1 && pcf_destination_addr >= 0 && packet_size_pcf > frag_threshold)
				{
				/* Determine number of fragments for the packet	*/
				/* and the size of the last fragment			*/							
				pcf_num_fragments =  (int) (packet_size_pcf / frag_threshold);
				pcf_remainder_size = packet_size_pcf - (pcf_num_fragments * frag_threshold);

				/* If the remainder size is non zero it means that the	*/
				/* last fragment is fractional but since the number 	*/
				/* of fragments is a whole number we need to transmit	*/	
				/* one additional fragment to ensure that all of the	*/
				/* data bits will be transmitted						*/
				if (pcf_remainder_size != 0)
					{
					pcf_num_fragments = pcf_num_fragments + 1;									 
					}
				}
			else
				{			
				/* If no fragments needed then number of	*/
				/* packets to be transmitted is set to 1	*/								
				pcf_num_fragments = 1;
				pcf_remainder_size = packet_size_pcf;
				}

			/* Make sure that fragment count is not more than 16.		*/
			if (pcf_num_fragments > 16)
				wlan_error_print ("Too many fragments are needed for higher layer packet! Maximum 16",
								  "fragments are allowed per MSDU. Disable \"Large Packet Processing\" and/or ",
								  "increase the fragmentation threshold (must be at least 256 bytes).");
		
			/* Storing Data packet id for debugging purposes.	*/			
			pcf_pkt_in_service = op_pk_id (hld_ptr->pkptr);		

			/* Insert packet to fragmentation buffer	*/					
			op_sar_segbuf_pk_insert (pcf_frag_buffer_ptr, hld_ptr->pkptr, 0);

			/* Printing out information to ODB.				*/
			if (wlan_trace_active == OPC_TRUE)
				{
				sprintf (msg_string, "Data packet " OPC_PACKET_ID_FMT " is removed from pcf queue.", pcf_pkt_in_service);
				sprintf	(msg_string1, "The queuing delay for data packet " OPC_PACKET_ID_FMT " is %fs.", 	
							pcf_pkt_in_service, current_time - hld_ptr->time_rcvd);	
				op_prg_odb_print_major (msg_string, msg_string1, OPC_NIL);
				}

			/* Store the arrival time of the pcf packet.	*/
			receive_time_pcf = hld_ptr->time_rcvd;
			
			/* Freeing up allocated memory for the data		*/
			/* packet removed from the higher layer queue.	*/
			op_prg_mem_free (hld_ptr);	
			}
		else
			{
			/* Destroy the dummy higher layer data entry	*/
			/* used earlier for searching.					*/
			op_prg_mem_free (hld_ptr);
			}

		/* Set active poll flag since poll will be			*/
		/* transmitted.										*/
		wlan_flags->active_poll = OPC_TRUE;

		/* Time to transmit fragment - Retries happen		*/
		/* automatically.									*/
		if (fresp_to_send == WlanC_Ack) 
			wlan_prepare_frame_to_send (WlanC_Data_A_P);
		else  
			wlan_prepare_frame_to_send (WlanC_Data_Poll);
		}
	else
		{
		/* The order of else if statements here is very important, as		*/
		/* the code uses it to enforce the proper preemption of various		*/
		/* valid frame sequences while preventing the preemption of others	*/

		/* If not PCF, an Ack needs to be sent for the data */
		/* prepare Ack for transmission	                    */
		if ((wlan_flags->polled == OPC_FALSE) && 
			(fresp_to_send == WlanC_Ack))
			{
			wlan_prepare_frame_to_send (fresp_to_send);
			
			/* Break the routine once Ack is prepared to transmit */
			FOUT;
			}

		/* Beacon transmission has priority unless we are in the middle of	*/
		/* transmitting fragments of a data packet.							*/
		else if (wlan_flags->tx_beacon == OPC_TRUE && 
				 (op_sar_buf_size (fragmentation_buffer_ptr) == 0 || short_retry_count + long_retry_count > 0 ||
				  op_sar_buf_size (fragmentation_buffer_ptr) == packet_size_dcf))
			{
			/* Reset any pending responses since beacon will terminate sequence anyway */
			fresp_to_send = WlanC_None;

			/* Prepare beacon frame to be transmitted */
			wlan_prepare_frame_to_send (WlanC_Beac);
			
			/* Break the routine once beacon prepared to transmit */
			FOUT;
			}

		/* DCF Transmission processing */
		
		/* Send a CTS frame if it is the type of frame we need to send a		*/
		/* response of.															*/
		else if (fresp_to_send == WlanC_Cts)
			{
			wlan_prepare_frame_to_send (fresp_to_send);
			
			/* Break the routine if Cts or Ack is already prepared to transmit.	*/
			FOUT;
			}
	
		/* If it is a retransmission then check which type of frame needs to be	*/
		/* retransmitted and then prepare and transmit that frame.				*/
		else if (short_retry_count + long_retry_count > 0)
			{
			/* If the last frame unsuccessfully transmitted was an RTS  or a 	*/
			/* CTS-to-self then transmit it again.								*/
			if ((last_frametx_type == WlanC_Rts || last_frametx_type == WlanC_Cts) && 
				wlan_flags->rts_sent == OPC_FALSE &&  wlan_flags->polled == OPC_FALSE)
				{
				wlan_prepare_frame_to_send (last_frametx_type);
				}

			/* If our last transmission was a data packet, then it means it was	*/
			/* not acknowledged. Restart the transmission process. Do the same	*/
			/* if we are resuming our retransmission after sending a beacon		*/
			/* frame or a management frame reporting end of CFP.				*/
			else if ((last_frametx_type == WlanC_Data   || last_frametx_type == WlanC_Beac ||
					  last_frametx_type == WlanC_Cf_End || last_frametx_type == WlanC_Cf_End_A) && wlan_flags->polled == OPC_FALSE)
				{			
				/* Check whether we need to start the retransmission with an	*/
				/* RTS message.													*/
				if (wlan_flags->frame_size_req_rts == OPC_TRUE && wlan_flags->rts_sent == OPC_FALSE)
					{
					/* Retransmit the RTS frame to again contend for the data .	*/
					wlan_prepare_frame_to_send (WlanC_Rts);		
					}
				
				/* If we are an ERP-STA, and we are not going to use an			*/
				/* 802.11/11b data rate for the transmission data, and there	*/
				/* are non-ERP STAs in the BSS, then we need to "use protection"*/
				/* by sending an RTS or CTS-to-self message.					*/
				else if (phy_type == WlanC_11g_PHY && wlan_flags->non_erp_present && 
					     (operational_speed > 5500000.0 && operational_speed != 11000000.0))
					{
					/* Use the "CTS-to-self" approach if the option is enabled.	*/
					/* Even it is enabled, switch using RTS/CTS for protection,	*/
					/* if our previous trials have failed as suggested in the	*/
					/* 802.11g standard (section 9.2.11), since the BSS can be	*/
					/* suffering from hidden node problem.						*/
					if (wlan_flags->cts_to_self && short_retry_count < 2)
						wlan_prepare_frame_to_send (WlanC_Cts);
					
					/* Otherwise initiate a RTS/CTS exchange as the protection	*/
					/* mechanism.												*/
					else
						wlan_prepare_frame_to_send (WlanC_Rts);
					}
				
				/* Just retransmit the data packet if no protection is needed.	*/
				else
					{
					wlan_prepare_frame_to_send (WlanC_Data);
					}
				}
			else
				{
				/* We continue with the retransmission process. Either we have	*/
				/* received the expected CTS for our last RTS before and now we	*/
				/* can retransmit our data frame, or we moved from DCF period	*/
				/* into PCF period and have been polled by the AP for			*/
				/* transmission. In case of PCF, also check whether we have an	*/
				/* ACK to append to our data packet.							*/
				if (fresp_to_send == WlanC_Ack && wlan_flags->polled == OPC_TRUE)
					wlan_prepare_frame_to_send (WlanC_Data_Ack);
				else 
					wlan_prepare_frame_to_send (WlanC_Data);
				}
			
			FOUT;
			}

		/* If higher layer queue is not empty then dequeue a packet	*/
		/* from the higher layer and insert it into fragmentation 	*/
		/* buffer check whether fragmentation and RTS-CTS exchange 	*/
		/* is needed  based on thresholds							*/
		/* Check if fragmentation buffer is empty. If it is empty   */
		/* then dequeue a packet from the higher layer queue.		*/ 
		else if ((op_prg_list_size (hld_list_ptr) != 0) && (op_sar_buf_size (fragmentation_buffer_ptr) == 0))
			{	
			/* Remove packet from higher layer queue. */
			hld_ptr = (WlanT_Hld_List_Elem *) op_prg_list_remove (hld_list_ptr, 0);
			
//自添加
			//$$$$$$$$$$$$$$$$$$ DSR $$$$$$$$$$$$$$$$$$$$$$$$
			op_pk_nfd_get((Packet*)(hld_ptr->pkptr),"Type",&data_packet_type);
			if (data_packet_type == DATA_PACKET_TYPE)
				 {
				 op_pk_nfd_get((Packet*)(hld_ptr->pkptr),"NextHop",&data_packet_dest);
			  	 op_pk_nfd_get((Packet*)(hld_ptr->pkptr),"DEST",&data_packet_final_dest);
				 }
			else
				 {
				 data_packet_dest=-1;
				 data_packet_final_dest=-1;
				 }
			//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//自添加结束
			
			/* Determine packet size to determine later whether fragmentation	*/
			/* and/or rts-cts exchange is needed.								*/
			packet_size_dcf = op_pk_total_size_get (hld_ptr->pkptr);
			
			/* Setting the address state variables.						*/				
			dcf_destination_addr = hld_ptr->destination_address;
			dcf_orig_source_addr = hld_ptr->origination_address;
			
			/* Store the higher layer protocol information.				*/
			hl_protocol_dcf = hld_ptr->protocol_type;
			
			/* Assign the next sequence number to the packet and		*/
			/* increment the counter.									*/
			packet_seq_control = packet_seq_counter << WLANC_FRAG_NUM_SIZE;   
			packet_seq_counter = (packet_seq_counter + 1) % WLANC_SEQ_NUM_WRAP_VALUE;
						
			/* Packet needs to be fragmented if it is more than			*/
			/* fragmentation threshold, provided fragmentation is		*/
			/* enabled. Broadcast packets are not fragmented regardless	*/
			/* of their sizes.											*/
			if (frag_threshold != -1 && (dcf_destination_addr >= 0 || (bss_flag == OPC_TRUE && ap_flag == OPC_BOOLINT_DISABLED)) && 
				packet_size_dcf > frag_threshold)
				{
				/* Determine number of fragments for the packet	*/
				/* and the size of the last fragment			*/							
				num_fragments =  (int) (packet_size_dcf / frag_threshold);
				remainder_size = packet_size_dcf - (num_fragments * frag_threshold);
				
				/* If the remainder size is non zero it means that the	*/
				/* last fragment is fractional but since the number 	*/
				/* of fragments is a whole number we need to transmit	*/
				/* one additional fragment to ensure that all of the	*/
				/* data bits will be transmitted						*/
				if (remainder_size != 0)
					num_fragments = num_fragments + 1;									 
				else
					/* Special case: data size is a multiple of the		*/
					/* fragment size, so all the fragments will be the	*/
					/* same size. To be consistent with other cases,	*/
					/* set remainder_size to the size of the last		*/
					/* fragment.										*/
					remainder_size = frag_threshold;
				}
			else
				{			
				/* If no fragments needed then number of packets to be	*/
				/* transmitted is set to 1								*/								
				num_fragments = 1;
				remainder_size = packet_size_dcf;
				}
			
			/* Make sure that fragment count is not more than 16.		*/
			if (num_fragments > 16)
				wlan_error_print ("Too many fragments are needed for higher layer packet! Maximum 16",
								  "fragments are allowed per MSDU. Disable \"Large Packet Processing\" and/or ",
								  "increase the fragmentation threshold (must be at least 256 bytes).");		
		
			/* Storing Data packet id for debugging purposes.	*/			
			pkt_in_service = op_pk_id (hld_ptr->pkptr);		
			
			/* Insert packet to fragmentation buffer	*/					
			op_sar_segbuf_pk_insert (fragmentation_buffer_ptr, hld_ptr->pkptr, 0);
			
			/* Printing out information to ODB.	*/
			if (wlan_trace_active == OPC_TRUE)
				{
				sprintf (msg_string, "Data packet " OPC_PACKET_ID_FMT " is removed from higher layer buffer", pkt_in_service);
				sprintf	(msg_string1, "The queuing delay for data packet " OPC_PACKET_ID_FMT " is %fs", 	
					pkt_in_service, current_time - hld_ptr->time_rcvd);	
				op_prg_odb_print_major (msg_string, msg_string1, OPC_NIL);
				}

			/* Store the arrival time of the packet.	*/
			receive_time_dcf = hld_ptr->time_rcvd;
			
			/* Free up allocated memory for the data packet removed from the higher		*/
			/* layer queue.																*/
			op_prg_mem_free (hld_ptr);
				
			/* Lower our data transmission rate, if it is an 11g data rate and we are	*/
			/* either an AP or a STA in an IBSS, and there are non-ERP STAs in our BSS	*/
			/* and our destination is one of them, so that it can decode our message.	*/
			if (phy_type == WlanC_11g_PHY && wlan_flags->non_erp_present && (bss_flag == OPC_FALSE || ap_flag == OPC_BOOLINT_ENABLED) &&
				data_tx_rate > 5500000.0 && data_tx_rate != 11000000.0)
				{
				/* Check whether the destination is 11g enabled. If this is a broadcast	*/
				/* transmission, use an 11b data rate since non-ERP STAs are present	*/
				/* the BSS.																*/			
				if (dcf_destination_addr < 0 || wlan_dest_is_11g_enabled (dcf_destination_addr) == OPC_FALSE)
					{
					/* Pick the highest 11b data rate that is lower than our regular	*/
					/* 11g data rate.													*/
					if (data_tx_rate > 11000000.0)
						operational_speed = 11000000.0;
					else if (data_tx_rate > 5500000.0)
						operational_speed = 5500000.0;
					}
				else
					operational_speed = data_tx_rate;
				}
			
			/* Send RTS if RTS is enabled and packet size is more than RTS threshold.	*/
			/* No RTS message is sent for broadcast packets regradless of their sizes.	*/
			if (rts_threshold != -1 && (dcf_destination_addr >= 0 || (bss_flag == OPC_TRUE && ap_flag == OPC_BOOLINT_DISABLED)) && 
				(packet_size_dcf + WLANC_MPDU_HEADER_SIZE) > rts_threshold && wlan_flags->polled == OPC_FALSE)			
				{
				/* Set the flag indicating that an RTS is needed for the current frame	*/
				/* due to its size.														*/
				wlan_flags->frame_size_req_rts = OPC_TRUE;
				
				/* Prepare RTS frame for transmission.									*/
				wlan_prepare_frame_to_send (WlanC_Rts);
				
				/* Break the routine as RTS is already prepared.						*/
				FOUT;
				}
			
			else
				{
				/* Reset the flag indicating an RTS was not necessary due to current	*/
				/* frame size.															*/
				wlan_flags->frame_size_req_rts = OPC_FALSE;
				
				/* If we are an ERP-STA, and we are not going to use an 802.11/11b data	*/
				/* rate for the transmission data, and there are non-ERP STAs in the	*/
				/* BSS,	then we need to "use protection" by sending an RTS or			*/
				/* CTS-to-self message.													*/
				if (phy_type == WlanC_11g_PHY && wlan_flags->non_erp_present && wlan_flags->polled == OPC_FALSE &&
					(operational_speed > 5500000.0 && operational_speed != 11000000.0))
					{
					/* Use the "CTS-to-self" approach and send a CTS message with		*/
					/* destination address set to our own address, if CTS-to-self		*/
					/* option is enabled or the data packet is a broadcast packet.		*/
					if (wlan_flags->cts_to_self || (dcf_destination_addr < 0 && (bss_flag == OPC_FALSE || ap_flag == OPC_BOOLINT_ENABLED)))
						wlan_prepare_frame_to_send (WlanC_Cts);
					
					/* Otherwise initiate a RTS/CTS exchange as the protection			*/
					/* mechanism.														*/
					else
						wlan_prepare_frame_to_send (WlanC_Rts);
					
					/* Exit the function.												*/
					FOUT;
					}
				}
			}
		
		/* Prepare data frame to transmit. First check whether the station	*/
		/* has been polled (if it is in CFP).								*/
		if (wlan_flags->polled == OPC_TRUE)
			{
			/* If there is no data to send select frame response			*/
			/* accordingly if we need to send an ACK back.					*/
			if (op_sar_buf_size (fragmentation_buffer_ptr) == 0)
				{
				if (fresp_to_send == WlanC_Ack) 
					wlan_prepare_frame_to_send (WlanC_Cf_Ack);
				else 
					wlan_prepare_frame_to_send (WlanC_Data_Null);
				}
			else  
				{
				/* We have data to respond to the poll. Also append the ACK	*/
				/* if we have an ACK to respond.							*/
				if (fresp_to_send == WlanC_Ack)
					wlan_prepare_frame_to_send (WlanC_Data_Ack);
				else 
					wlan_prepare_frame_to_send (WlanC_Data);
				}
			}		
		else
			{
			/* This is a normal DCF transmission. Prepare the frame for		*/
			/* transmission.												*/
			wlan_prepare_frame_to_send (WlanC_Data);
			}
		}
	
	FOUT;
	}

static double		
wlan_non_11b_plcp_overhead_compute (OpT_Packet_Size mpdu_length, double data_rate)
	{
	/** This function is called by MACs that operate either in 11a or in	**/
	/** 11g mode. The function decides on the type of PLCP preamble/header	**/
	/** and	accordingly computes the total size of the PLCP overhead in		**/
	/** bits for the transmission whose MPDU length and data rate are		**/
	/** provided. The function returns this computed overhead.				**/
	FIN (wlan_non_11b_plcp_overhead_compute (mpdu_length, data_rate));
	
	/* Use the OFDM preamble if we are using an OFDM data rate.				*/
	if (data_rate > 5500000.0 && data_rate != 11000000.0)
		{
		/* Compute and the return the OFDM PLCP overhead.					*/
		FRET (wlan_plcp_overhead_ofdm_compute (mpdu_length, data_rate));
		}
	
	/* Use the short preamble if the data rate is higher than 1 Mbps and	*/
	/* there are no non-ERP STAs in our (I)BSS (we assume that non-ERP STAs	*/
	/* don't support short preambles).										*/
	else if (data_rate != 1000000.0 && wlan_flags->non_erp_present == OPC_FALSE)
		{
		FRET (WLANC_PLCP_OVERHEAD_DSSS_SHORT);
		}
	
	/* Otherwise use the long preamble.										*/
	else
		{
		FRET (WLANC_PLCP_OVERHEAD_DSSS_LONG);
		}
	}

static double		
wlan_plcp_overhead_ofdm_compute (OpT_Packet_Size mpdu_length, double data_rate)
	{
	int		N_dbps, data_size, padding;
	
	/** This function computes and returns the total PLCP overhead			**/
	/** (preamble + header) in seconds for the given MPDU length and data	**/
	/** rate for a transmission that will use the OFDM technology specified	**/
	/** in the 802.11a standard.											**/ 
	FIN (wlan_plcp_overhead_ofdm_compute (mpdu_length, data_rate));
	
	/* Compute the number of padding bits. First find out the total size we	*/
	/* are trying to achieve.												*/
	N_dbps    = (int) (data_rate * 4.0 / 1.0E06);
	data_size = N_dbps * (int) ceil ((double)(mpdu_length + WLANC_SERVICE_AND_TAIL_SIZE) / N_dbps);
	
	/* The size of the padding is the difference between the actual size	*/
	/* and the target size.													*/
	padding = data_size - ((int) mpdu_length + WLANC_SERVICE_AND_TAIL_SIZE);
	
	/* Compute and return the total PLCP overhead by adding the overhad of	*/
	/* SERVICE, Tail and Padding bits to the rest of the PLCP delay.		*/
	if (data_rate == control_data_rate)
		{
		FRET (plcp_overhead_control + (WLANC_SERVICE_AND_TAIL_SIZE + padding) / data_rate);
		}
	else
		{
		FRET (plcp_overhead_data + (WLANC_SERVICE_AND_TAIL_SIZE + padding) / data_rate);
		}
	}
		
static Boolean
wlan_dest_is_11g_enabled (OpT_Int64 dest_mac_addr)
	{
	WlanT_Peer_Info*	sta_info_ptr;
	
	/** This function returns TRUE or FALSE based on whether the STA whose	**/
	/** MAC address is provided is an ERP or a non-ERP STA (11g enabled or	**/
	/** not). 																**/
	FIN (wlan_dest_is_11g_enabled (dest_mac_addr));
	
	/* Get the destination's record.										*/
	sta_info_ptr =  (WlanT_Peer_Info *) prg_bin_hash_table_item_get (peer_info_hash_tbl, (void *) &(dest_mac_addr));
	if (sta_info_ptr != OPC_NIL)
		{
		/* Return the ERP capability information of the destination peer.	*/
		FRET (sta_info_ptr->is_erp);
		}
	
	/* If the destination is not found in our BSS, then assume that it		*/
	/* supports 11g, so that we don't waste even more time by using a low	*/
	/* data rate.															*/
	FRET (OPC_TRUE);
	}

static void 
wlan_prepare_frame_to_send (WlanT_Mac_Frame_Type frame_type)
	{
	Packet*							seg_pkptr;
	OpT_Packet_Size					tx_datapacket_size;
	WlanT_Mac_Frame_Type			type;
	int								i;
	OpT_Int64						destination_addr;
	OpT_Int64						orig_source_addr;
	int								add_beacon_size;
	double							tx_data_rate;
	double							duration, mac_delay;
	double							total_pk_size;
	double							tx_end_time, tx_delay;
	double							total_plcp_overhead;
	double							bulk_size;
	double							total_frame_size;
	WlanT_Data_Header_Fields*		pk_dhstruct_ptr;
	WlanT_Control_Header_Fields*	pk_chstruct_ptr;
	WlanT_Beacon_Body_Fields*		pk_bbstruct_ptr;
	Packet*							wlan_transmit_frame_ptr;
	char							msg_string [120];
	char							frame_type_str [32];
	
	/** Prepare frames to transmit by setting appropriate fields in the 	**/
    /** packet format for Data,Cts,Rts or Ack.  If data or Rts packet needs **/
    /** to be retransmitted then the copy of the packet is resent.          **/
	FIN (wlan_prepare_frame_to_send (frame_type));

	/* First initialize the transmission data rate to the lowest supported	*/
	/* data rate, which is the data rate used for control frames.			*/
	tx_data_rate = control_data_rate;

	/* Determine destination and original source address information based	*/
	/* on the type of the transmission (PCF data transmission by AP or not).*/
	if (wlan_flags->pcf_active == OPC_TRUE)
		{
		destination_addr = pcf_destination_addr;
		orig_source_addr = pcf_orig_source_addr;
		}
	else
		{
		destination_addr = dcf_destination_addr;
		orig_source_addr = dcf_orig_source_addr;
		}
		
	/* It this is a CP period and the frame to be transmitted is a data/ACK.*/
	if ((wlan_flags->pcf_active == OPC_FALSE) && 
	   ((frame_type == WlanC_Data) || (frame_type == WlanC_Data_Ack)))
		{
		/* Adjust the transmission data rate based on the operational speed.*/
		tx_data_rate = operational_speed;

		/* Set the variable which keeps track of the last transmitted frame.	*/
		last_frametx_type = frame_type;
					
		/* If it is a retransmission of a packet. Obtain the frame from the 	*/
		/* the copy pointer which was stored during the previous transmission	*/
		if ((short_retry_count + long_retry_count > 0) && (wlan_transmit_frame_copy_ptr != OPC_NIL))
			{
			/* If it is a retransmission then just transmit the previous frame	*/			
			wlan_transmit_frame_ptr = op_pk_copy (wlan_transmit_frame_copy_ptr);

			/* Reset header type in case Ack status has changed for frame */
			op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_DATA_TYPE_FD, frame_type, OPC_FIELD_SIZE_UNCHANGED);

			/* If retry count is non-zero means that the frame is a */
			/* retransmission of the last transmitted frame			*/
			op_pk_fd_access_ptr (wlan_transmit_frame_ptr, WLANC_DATA_HEADER_FD, (void **) &pk_dhstruct_ptr);
			pk_dhstruct_ptr->retry = OPC_TRUE;
			
			/* Reset more_data bit in case queue status has changed since last transmission			*/
			/* If this STA has been polled, and there are additional packets remaining				*/
			if ((wlan_flags->polled == OPC_TRUE) && (op_prg_list_size (hld_list_ptr) != 0))
				{
				/* Set more data bit to tell AP that STA has more packets */
				pk_dhstruct_ptr->more_data = 1;
				}

			/* Printing out information to ODB.	*/
			if (wlan_trace_active == OPC_TRUE)
				{
				sprintf (msg_string, "Data fragment %d for packet " OPC_PACKET_ID_FMT " is retransmitted.",
					     pk_dhstruct_ptr->sequence_control & WLANC_FRAG_NUM_BIT_MASK, pkt_in_service);							
				op_prg_odb_print_major (msg_string, OPC_NIL);
				}					

			/* Calculate NAV duration till the channel will be occupied by 	*/
			/* station. The duration is SIFS time plus the ACK frame time,	*/
			/* which the station needs in response to the data frame (note:	*/
			/* no need to check for broadcast packets, since for broadcast	*/
			/* packets the encapsulating if condition will be never true).	*/
			duration = sifs_time + TXTIME_CTRL (WLANC_ACK_LENGTH);		
			
			/* Since the number of fragments for the last transmitted frame is	*/
			/* already decremented, there will be more fragments to transmit  	*/
			/* if number of fragments is more than zero.					  	*/
			if (num_fragments != 1)	
				{
				/* If more fragments need to be transmitted then the station 	*/
				/* need to compute the duration until the receipt of the       	*/
				/* the acknowledgement for the next fragment. 224 bits (header	*/
				/* size) is the length of the control fields in the data  		*/
				/* frame and needs to be accounted in the duration calculation.	*/
				if (num_fragments == 2)
					tx_datapacket_size = remainder_size + WLANC_MPDU_HEADER_SIZE;
				else
					tx_datapacket_size = frag_threshold + WLANC_MPDU_HEADER_SIZE;
				duration = 2 * duration + sifs_time + TXTIME_DATA (tx_datapacket_size);
				}

			/* Set the type of the expected response to "ACK".	*/			
			expected_frame_type = WlanC_Ack;
			
			/* Station update its own nav_duration during CP   	*/
			/* NAV should be updated only during the CP period 	*/
			/* During CFP NAV duration is updated only during	*/
			/* the transmission of the beacon frames			*/
			if (cfp_ap_medium_control == OPC_FALSE)
				nav_duration = current_time + duration + (double) (op_pk_total_size_get (wlan_transmit_frame_ptr)) / operational_speed ;
			}
		else
			{
			/* Creating transmit data packet type.							*/
			wlan_transmit_frame_ptr = op_pk_create_fmt ("wlan_mac");
				
			/* Prepare data frame fields for transmission.					*/		
			pk_dhstruct_ptr = wlan_pk_dhstruct_create ();

			type = frame_type;
			
			pk_dhstruct_ptr->retry = OPC_FALSE;				
			pk_dhstruct_ptr->order = 1;
			pk_dhstruct_ptr->sequence_control = packet_seq_control;
			pk_dhstruct_ptr->hl_protocol = hl_protocol_dcf;

			/* Calculate nav duration till the channel will be occupied by  */
			/* station. The duration is SIFS time plus the ack frame time   */
			/* which the station needs in response to the data frame. For	*/
			/* broadcast packets, the duration is zero since they are not	*/
			/* acknowledged.												*/
			if (destination_addr >= 0 || (bss_flag == OPC_TRUE && ap_flag == OPC_BOOLINT_DISABLED))
				duration = sifs_time + TXTIME_CTRL (WLANC_ACK_LENGTH);
			else
				duration = 0.0;
			
			/* If there is more than one fragment to transmit then remove	*/
			/* fragmentation threshold size length of data from the buffer	*/
			/* for transmission.											*/
			if  (num_fragments > 1)
				{
				/* Remove next fragment from the fragmentation buffer for 	*/
				/* transmission and set the appropriate fragment number.  	*/
				seg_pkptr = op_sar_srcbuf_seg_remove (fragmentation_buffer_ptr, frag_threshold);
			
				/* Indicate in transmission frame that more fragments need	*/
				/* to be sent.												*/
				pk_dhstruct_ptr->more_frag = 1;
				
				/* Since more fragments need to be transmitted then the		*/
				/* station need to broadcast the time until the receipt of	*/
				/* the acknowledgement for the next fragment. 224 bits		*/
				/* (header size) is the length of control fields in the		*/
				/* data frame and need to be accounted for in the duration	*/
				/* calculation.												*/
				if (num_fragments == 2)
					tx_datapacket_size = remainder_size + WLANC_MPDU_HEADER_SIZE;
				else
					tx_datapacket_size = frag_threshold + WLANC_MPDU_HEADER_SIZE;
				duration = 2 * duration + sifs_time + TXTIME_DATA (tx_datapacket_size);
						
				/* Printing out information to ODB.							*/
				if (wlan_trace_active == OPC_TRUE)
					{
					sprintf (msg_string, "Data fragment %d for packet " OPC_PACKET_ID_FMT " is transmitted.", 
						packet_seq_control & WLANC_FRAG_NUM_BIT_MASK, pkt_in_service);							
					op_prg_odb_print_major (msg_string, OPC_NIL);
					}

				/* Set the sequence control number for next fragment to be	*/
				/* transmitted.												*/
				packet_seq_control++;    	
				}
			else
				{
				/* Remove the last fragment from the fragmentation buffer for 		*/
				/* transmission and disable more fragmentation bit.					*/												
				seg_pkptr = op_sar_srcbuf_seg_remove (fragmentation_buffer_ptr, remainder_size);					
				pk_dhstruct_ptr->more_frag = 0;
				
				/* Printing out information to ODB.	*/
				if (wlan_trace_active == OPC_TRUE)
					{
					sprintf (msg_string, "Data fragment %d for packet " OPC_PACKET_ID_FMT " is transmitted.", 
						packet_seq_control & WLANC_FRAG_NUM_BIT_MASK, pkt_in_service);								
					op_prg_odb_print_major (msg_string, OPC_NIL);
					}
				}	

			/* Setting the Header field structure.	*/

			/** if this is the CF period and the STA has been polled	**/
			/** then set the duration to the standard value.			**/
			if (wlan_flags->polled == OPC_TRUE)
				{
				/* Duration should be set to 32768 during CFP.										*/
				duration = 32768.0;
				pk_dhstruct_ptr->duration  = duration;
				}
			else
				{
				/* This is the CP, so set duration field.											*/
				pk_dhstruct_ptr->duration  = duration;
				}
			
			pk_dhstruct_ptr->address1  = destination_addr;
			pk_dhstruct_ptr->address2  = my_address;

			/* In the BSS network the Data frame is going from AP to STA then fromds bit is set and	*/
			/* address3 contains the address of the MAC that is the original sender of the packet.	*/
    		if (ap_flag == OPC_BOOLINT_ENABLED)
				{
				pk_dhstruct_ptr->fromds	= 1;
				pk_dhstruct_ptr->address3 = orig_source_addr;
				}	
			else
				pk_dhstruct_ptr->fromds	= 0;

			/* if in the BSS network the Data frame is going from sta to AP then tods bit is set.	*/					
    		if (bss_flag == OPC_TRUE && ap_flag == OPC_BOOLINT_DISABLED)
				{
				pk_dhstruct_ptr->tods = 1;
				
				/* If Infrastructure BSS then the immediate destination will be Access point, which */
				/* then forward the frame to the appropriate destination.							*/
				pk_dhstruct_ptr->address1 = ap_mac_address ;
				pk_dhstruct_ptr->address3 = destination_addr;
				}
			else
				{
				pk_dhstruct_ptr->tods = 0;
				}

			/* If this STA has been polled, and there are additional packets remaining				*/
			if ((wlan_flags->polled == OPC_TRUE) && (op_prg_list_size (hld_list_ptr) != 0))
				{
				/* Set more data bit to tell AP that STA has more packets */
				pk_dhstruct_ptr->more_data = 1;
				}
			
			/* If we are sending the first fragment of the data fragment for the first	*/
			/* time, then this is the end of media access duration, hence we must		*/
			/* update the media access delay statistics.								*/
			if ((pk_dhstruct_ptr->sequence_control & WLANC_FRAG_NUM_BIT_MASK) == 0)
				{
				mac_delay = current_time - receive_time_dcf;
				op_stat_write (media_access_delay, mac_delay);
				op_stat_write (global_mac_delay_handle, mac_delay);
				}
			
			/* Populate the packet fields.								*/
			op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_DATA_TYPE_FD,   type,           OPC_FIELD_SIZE_UNCHANGED);
			op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_DATA_ACCEPT_FD, OPC_TRUE,       OPC_FIELD_SIZE_UNCHANGED);
			op_pk_fd_set_pkid  (wlan_transmit_frame_ptr, WLANC_DATA_PKID_FD,   pkt_in_service, OPC_FIELD_SIZE_UNCHANGED);
				
			/* Set the frame control field and nav duration.		   	*/
			op_pk_fd_set_ptr (wlan_transmit_frame_ptr, WLANC_DATA_HEADER_FD, pk_dhstruct_ptr, OPC_FIELD_SIZE_UNCHANGED,	
						      wlan_pk_dhstruct_copy, wlan_pk_dhstruct_destroy, sizeof (WlanT_Data_Header_Fields));

			/* The actual data is placed in the Frame Body field.		*/
			op_pk_fd_set_pkt (wlan_transmit_frame_ptr, WLANC_DATA_BODY_FD, seg_pkptr, OPC_FIELD_SIZE_UNCHANGED);

			/* Add some bulk to the packet to model the transmission	*/
			/* delay of PLCP fields accurately which are always			*/
			/* transmitted at 1 Mbps regardless of the actual data rate	*/
			/* used for data frames.									*/
			total_plcp_overhead = PLCP_OVERHEAD_DATA ((int) op_pk_total_size_get (wlan_transmit_frame_ptr) - WLANC_DEFAULT_PLCP_OVERHEAD);
			bulk_size = total_plcp_overhead * operational_speed - WLANC_DEFAULT_PLCP_OVERHEAD;
			op_pk_bulk_size_set (wlan_transmit_frame_ptr, (OpT_Packet_Size) bulk_size);
		
			/* Expect acknowledgement only for directed frames.			*/
			if (pk_dhstruct_ptr->address1 < 0)
				{
				expected_frame_type = WlanC_None;
				
				/* Reset the retry count because we won't await an ACK.	*/
				/* The retry count can be non-zero even for a broadcast	*/
				/* frame since it can be proceeded by a CTS-to-self		*/
				/* frame in an 11g WLAN, which may have been			*/
				/* retransmitted.										*/
				short_retry_count = 0;
				
				/* Due to possible earlier use of CTS-to-self frame		*/
				/* exchange, reset the rts_sent flag.					*/
				wlan_flags->rts_sent = OPC_FALSE;
				
				/* Transmission of a broadcast frame is always assumed	*/
				/* successful. Hence, set the flag for CW backoff.		*/
				wlan_flags->cw_required = OPC_TRUE;
				
				/* This frame won't be retransmitted.					*/
				wlan_transmit_frame_copy_ptr = (Packet *) OPC_NIL;

				/* Since the transmission of the higher layer packet is	*/
				/* complete, update the queue size information and		*/
				/* statistic.											*/
				total_hlpk_num--;
				total_hlpk_size -= packet_size_dcf;
				op_stat_write (hl_packets_rcvd, (double) total_hlpk_num);
				}
			else
				{
				/* Ack frame is expected in response to data frame.		*/
				expected_frame_type = WlanC_Ack;
				
				/* Make a copy of the frame in case a retransmission is	*/
				/* needed.												*/
				wlan_transmit_frame_copy_ptr = op_pk_copy (wlan_transmit_frame_ptr);
				}

		    /* Station update of its own nav_duration.					*/
			if (cfp_ap_medium_control == OPC_FALSE)
				nav_duration = current_time + duration + (double) (op_pk_total_size_get (wlan_transmit_frame_ptr)) / operational_speed ;
			}
		
		/* Place the transmission data rate and physical layer			*/
		/* technology information into the packet.						*/
		wlan_frame_tx_phy_info_set (wlan_transmit_frame_ptr, tx_data_rate, phy_type, phy_char_flag);
	
		/* Update the data traffic sent statistics.						*/
		total_pk_size = (double) op_pk_total_size_get (wlan_transmit_frame_ptr);
		op_stat_write (data_traffic_sent_handle_inbits, total_pk_size);
		op_stat_write (data_traffic_sent_handle, 1.0);

		/* Write a value of 0 for the end of transmission.				*/
		tx_end_time = current_time + total_pk_size / operational_speed;
		op_stat_write_t (data_traffic_sent_handle_inbits, 0.0, tx_end_time);
		op_stat_write_t (data_traffic_sent_handle, 0.0, tx_end_time);
		
		/* We can be sending this data message as a response to a CTS message	*/
		/* we received. Therefore reset the "frame respond to send" variable.	*/
		fresp_to_send = WlanC_None;
		
		/* If there is nothing in the higher layer data queue and fragmentation buffer	*/
		/* then disable the data frame flag which will indicate to the station to wait	*/
		/* for the higher layer packet.													*/
		if (op_prg_list_size (hld_list_ptr) == 0 && op_sar_buf_size (fragmentation_buffer_ptr) == 0)
			wlan_flags->data_frame_to_send = OPC_FALSE;			
		}

	/* If this is a contention free period and need to send a data/ack/poll.	*/
	else if ((wlan_flags->pcf_active == OPC_TRUE) &&
			((frame_type == WlanC_Data)	|| (frame_type == WlanC_Data_Ack) || 
			(frame_type == WlanC_Data_Poll)	|| (frame_type == WlanC_Data_A_P))) 
		{
		/* Preserve the frame type being transmitted */
		last_frametx_type = frame_type;
		
		/* Adjust the transmission data rate based on the operational speed.*/
		tx_data_rate = operational_speed;

		/* Set active poll flag if this is a poll frame */
		if ((frame_type == WlanC_Data_Poll)	|| (frame_type == WlanC_Data_A_P))
			{
			wlan_flags->active_poll = OPC_TRUE;
			}
		
		/* If it is a retransmission of a packet then no need 	*/
        /* to prepare data frame.							    */
		if (pcf_retry_count == 0)
			{
			/* Creating transmit data packet type.							*/
			wlan_transmit_frame_ptr = op_pk_create_fmt ("wlan_mac");
				
			/* Prepare data frame fields for transmission.					*/		
			pk_dhstruct_ptr = wlan_pk_dhstruct_create ();

			pk_dhstruct_ptr->retry = OPC_FALSE;				
			pk_dhstruct_ptr->order = 1;
			pk_dhstruct_ptr->sequence_control = packet_seq_control;
			pk_dhstruct_ptr->hl_protocol = hl_protocol_pcf;

			/* If there is more than one fragment to transmit and there are  	*/
			/* equal sized fragments then remove fragmentation threshold size	*/
			/* length of data from the buffer for transmission.					*/
			if  ((pcf_num_fragments > 1) || (pcf_remainder_size == 0))
				{
				/* Remove next fragment from the fragmentation buffer for 	*/
				/* transmission and set the appropriate fragment number.  	*/
				seg_pkptr = op_sar_srcbuf_seg_remove (pcf_frag_buffer_ptr, frag_threshold);
		
				/* Indicate in transmission frame that more fragments need to be sent	*/
				/* if more than one fragments are left								 	*/
				if (pcf_num_fragments != 1)	
					{
					pk_dhstruct_ptr->more_frag = 1;
					}
				else
					{
					/* If no more fragments to transmit then set more fragment field to be 0 */
					pk_dhstruct_ptr->more_frag = 0;
					}
						
				/* Printing out information to ODB.	*/
				if (wlan_trace_active == OPC_TRUE)
					{
					sprintf (msg_string, "Data fragment %d for packet " OPC_PACKET_ID_FMT " is transmitted.",
						packet_seq_control & WLANC_FRAG_NUM_BIT_MASK, pcf_pkt_in_service);							
					op_prg_odb_print_major (msg_string, OPC_NIL);
					}

				/* Setting packet sequence control number for the next fragment to be	*/
				/* transmitted.															*/
				packet_seq_control++;    	
				}
			else
				{
				/* Remove last fragments (if any left) from the fragmentation buffer for */
				/* transmission and disable more fragmentation bit.				         */
				seg_pkptr = op_sar_srcbuf_seg_remove (pcf_frag_buffer_ptr, pcf_remainder_size);					

				pk_dhstruct_ptr->more_frag = 0;

				/* Printing out information to ODB.	*/
				if (wlan_trace_active == OPC_TRUE)
					{
					sprintf (msg_string, "Data fragment %d for packet " OPC_PACKET_ID_FMT " is transmitted.",
							 packet_seq_control & WLANC_FRAG_NUM_BIT_MASK, pcf_pkt_in_service);								
					op_prg_odb_print_major (msg_string, OPC_NIL);
					}
				}	

			/* Set duration field */
			/* During CFP the duration field should read 32768. (Section 7.1.3.2 of spec) */
			duration = 32768.0;

			/* Setting the Header field structure.	*/
			pk_dhstruct_ptr->duration  = duration;
			pk_dhstruct_ptr->address1  = destination_addr;
			pk_dhstruct_ptr->address2  = my_address;

			/* In the BSS network the Data frame is going from AP to STA then fromds bit is set and	*/
			/* address3 contains the address of the MAC that is the original sender of the packet.	*/
    		if (ap_flag == OPC_BOOLINT_ENABLED)
				{
				pk_dhstruct_ptr->fromds	= 1;
				pk_dhstruct_ptr->address3 = orig_source_addr;
				}
			else
				{
				pk_dhstruct_ptr->fromds = 0;
				}

			/* if in the BSS network the Data frame is going from sta to AP then tods bit is set.	*/					
    		if ((bss_flag == OPC_TRUE) && (ap_flag == OPC_BOOLINT_DISABLED))
				{
				pk_dhstruct_ptr->tods = 1;

				/* If Infrastructure BSS then the immediate destination will be Access point, which */
				/* then forward the frame to the appropriate destination.							*/
				pk_dhstruct_ptr->address1 = ap_mac_address;
				pk_dhstruct_ptr->address3 = destination_addr;
				}
			else
				{
				pk_dhstruct_ptr->tods = 0;
				}
	
			/* If we are sending the first fragment of the data fragment for the first	*/
			/* time, then this is the end of media access duration, hence we must		*/
			/* update the media access delay statistics.								*/
			if (packet_size_pcf == op_pk_total_size_get (seg_pkptr) + op_sar_buf_size (pcf_frag_buffer_ptr))
				{
				mac_delay = current_time - receive_time_pcf;
			
				op_stat_write (media_access_delay, mac_delay);
				op_stat_write (global_mac_delay_handle, mac_delay);
				}
			
			op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_DATA_TYPE_FD,   frame_type,         OPC_FIELD_SIZE_UNCHANGED);
			op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_DATA_ACCEPT_FD, OPC_TRUE,           OPC_FIELD_SIZE_UNCHANGED);
			op_pk_fd_set_pkid  (wlan_transmit_frame_ptr, WLANC_DATA_PKID_FD,   pcf_pkt_in_service, OPC_FIELD_SIZE_UNCHANGED);
				
			/* Set the frame control field.				*/
			op_pk_fd_set_ptr (wlan_transmit_frame_ptr, WLANC_DATA_HEADER_FD, pk_dhstruct_ptr, OPC_FIELD_SIZE_UNCHANGED,	
						      wlan_pk_dhstruct_copy, wlan_pk_dhstruct_destroy, sizeof (WlanT_Data_Header_Fields));

			/* The actual data is placed in the Frame Body field	*/
			op_pk_fd_set_pkt (wlan_transmit_frame_ptr, WLANC_DATA_BODY_FD, seg_pkptr, OPC_FIELD_SIZE_UNCHANGED);

			/* Add some bulk to the packet to model the transmission delay	*/
			/* of PLCP fields accurately which are always transmitted at	*/
			/* 1 Mbps regardless of the actual data rate used for data		*/
			/* frames.														*/
			total_plcp_overhead = PLCP_OVERHEAD_DATA ((int) op_pk_total_size_get (wlan_transmit_frame_ptr) - WLANC_DEFAULT_PLCP_OVERHEAD);
			bulk_size = total_plcp_overhead * operational_speed - WLANC_DEFAULT_PLCP_OVERHEAD;
			op_pk_bulk_size_set (wlan_transmit_frame_ptr, (OpT_Packet_Size) bulk_size);
		
			/* Make copy of the frame before transmission	*/
			wlan_pcf_transmit_frame_copy_ptr = op_pk_copy (wlan_transmit_frame_ptr);
			}
		else
			{
			/* If it is a retransmission then just transmit the previous frame	*/			
			wlan_transmit_frame_ptr = op_pk_copy (wlan_pcf_transmit_frame_copy_ptr);

			/* If retry count is non-zero means that the frame is a */
			/* retransmission of the last transmitted frame			*/
			op_pk_fd_access_ptr (wlan_transmit_frame_ptr, WLANC_DATA_HEADER_FD, (void **) &pk_dhstruct_ptr);
			pk_dhstruct_ptr->retry = OPC_TRUE;

			/* Reset header type in case Ack status has changed for frame */
			op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_DATA_TYPE_FD, frame_type, OPC_FIELD_SIZE_UNCHANGED);
			
			/* read back duration field for debug stuff */
			duration =	pk_dhstruct_ptr->duration;

			/* Printing out information to ODB.	*/
			if (wlan_trace_active == OPC_TRUE)
				{
				sprintf (msg_string, "Data fragment %d for packet " OPC_PACKET_ID_FMT " is retransmitted.", 
						pk_dhstruct_ptr->sequence_control & WLANC_FRAG_NUM_BIT_MASK, pcf_pkt_in_service);							
				op_prg_odb_print_major (msg_string, OPC_NIL);
				}					
			}
			
		/* Place the transmission data rate and physical layer			*/
		/* technology information into the packet.						*/
		wlan_frame_tx_phy_info_set (wlan_transmit_frame_ptr, tx_data_rate, phy_type, phy_char_flag);
	
		/* Update the data traffic sent statistics.						*/
		total_pk_size = (double) op_pk_total_size_get (wlan_transmit_frame_ptr);
		op_stat_write (data_traffic_sent_handle_inbits, total_pk_size);
		op_stat_write (data_traffic_sent_handle, 1.0);

		/* Write a value of 0 for the end of transmission.				*/
		tx_end_time = current_time + total_pk_size / operational_speed;
		op_stat_write_t (data_traffic_sent_handle_inbits, 0.0, tx_end_time);
		op_stat_write_t (data_traffic_sent_handle, 0.0, tx_end_time);

		/* Only expect Acknowledgement for directed frames.				*/
		if (destination_addr < 0)
			expected_frame_type = WlanC_None;
		else
			/* ACK frame is expected in response to data frame.			*/
			expected_frame_type = WlanC_Ack;
		
		/* Reset the "frame to respond" variable since we have piggy-	*/
		/* backed an ACK to our message if we had to send one.			*/
		fresp_to_send = WlanC_None;
		}
	
	else if	(frame_type == WlanC_Ack)			
		{	
		if ((pcf_flag == OPC_BOOLINT_ENABLED) || (wlan_flags->pcf_active == OPC_TRUE))
			last_frametx_type = frame_type;
				
		/* Preparing acknowledgement frame in response to the data frame	*/
		/* received from the remote stations.								*/
		
		/* Since an ACK is a control response frame, adjust its				*/
		/* transmission rate based on the data rate of the data frame we	*/
		/* are ACKing, if operating in an 11a or all-11g BSS. Otherwise use	*/
		/* 1 Mbps, the mandatory PHY rate of 802.11/11b.					*/
		if (phy_type == WlanC_11a_PHY || (phy_type == WlanC_11g_PHY && wlan_flags->non_erp_present == OPC_FALSE))
			tx_data_rate = wlan_ctrl_response_drate_determine (rcvd_frame_drate);
		
		/* Creating ACK packet format type.									*/
		wlan_transmit_frame_ptr = op_pk_create_fmt ("wlan_control");

		/* Adjust the packet size if necessary to model the PLCP overhead	*/
		/* accurately, which is physical layer technology dependent. The	*/
		/* default value is set for infra-red technology.					*/
		if (phy_char_flag != WlanC_Infra_Red)
			{
			bulk_size = PLCP_OVERHEAD_CTRL (WLANC_ACK_LENGTH) * tx_data_rate - WLANC_DEFAULT_PLCP_OVERHEAD;
			op_pk_bulk_size_set (wlan_transmit_frame_ptr, (OpT_Packet_Size) bulk_size);
			}

		/* Setting ack frame fields.										*/
		pk_chstruct_ptr = wlan_pk_chstruct_create ();
   		
		type = WlanC_Ack;   		
		pk_chstruct_ptr->retry = duplicate_entry;

		/* If there are more fragments to transmit then broadcast the remaining duration for which	*/
		/* the station will be using the channel.													*/
		if ((wlan_flags->duration_zero == OPC_FALSE) || (pcf_flag == OPC_BOOLINT_DISABLED))
			duration = nav_duration - (current_time + TXTIME_CTRL (WLANC_ACK_LENGTH));
		else 
			duration = 0;
		
		pk_chstruct_ptr->duration = duration;

		/* Destination station address.	*/
		pk_chstruct_ptr->rx_addr = remote_sta_addr;

		/* Setting Ack type.	*/
		op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_CNTL_TYPE_FD, type, OPC_FIELD_SIZE_UNCHANGED);
			
		/* Setting the accept field to true, meaning the frame is a good frame.	*/
		op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_CNTL_ACCEPT_FD, OPC_TRUE, OPC_FIELD_SIZE_UNCHANGED);

		op_pk_fd_set_ptr (wlan_transmit_frame_ptr, WLANC_CNTL_HEADER_FD, pk_chstruct_ptr, OPC_FIELD_SIZE_UNCHANGED, 
						  wlan_pk_chstruct_copy, wlan_pk_chstruct_destroy, sizeof (WlanT_Control_Header_Fields));

		/* Place the transmission data rate and physical layer			*/
		/* technology information into the packet.						*/
		wlan_frame_tx_phy_info_set (wlan_transmit_frame_ptr, tx_data_rate, phy_type, phy_char_flag);
	
		/* Since no frame is expected, the expected frame type field */
		/* to nil.                                                   */
		expected_frame_type = WlanC_None;	
	
		/* Once Ack is transmitted in response to Data frame then set the frame		*/
		/* response indicator to none frame as the response is already generated	*/
		fresp_to_send = WlanC_None;			

		/* Printing out information to ODB.	*/
		if (wlan_trace_active == OPC_TRUE)
			{
			op_prg_odb_print_major ("Ack is being transmitted for data packet received", OPC_NIL);
			}

		/* Update the control traffic sent statistics.					*/
		total_pk_size = (double) op_pk_total_size_get (wlan_transmit_frame_ptr);
		op_stat_write (ctrl_traffic_sent_handle_inbits, total_pk_size);
		op_stat_write (ctrl_traffic_sent_handle, 		1.0);

		/* Write a value of 0 for the end of transmission.				*/
		tx_end_time = current_time + total_pk_size / tx_data_rate;
		op_stat_write_t (ctrl_traffic_sent_handle_inbits, 0.0, tx_end_time);
		op_stat_write_t (ctrl_traffic_sent_handle, 		  0.0, tx_end_time);
		}
	
	else if (frame_type == WlanC_Rts)
		{		
		/* Creating Rts packet format type.									*/
		wlan_transmit_frame_ptr = op_pk_create_fmt ("wlan_control");

		/* Initializing RTS frame fields.									*/
		pk_chstruct_ptr = wlan_pk_chstruct_create ();
		
		/* Type of frame */
	   	type = WlanC_Rts;   						

		/* if in the infrastructure BSS network then the immediate recipient for the transmitting	*/
		/* station will always be an Access point. Otherwise the frame is directly sent to the 		*/
		/* final destination.																		*/
    	if ((bss_flag == OPC_TRUE) && (ap_flag == OPC_BOOLINT_DISABLED))
			{
			/* If Infrastructure BSS then the immediate destination will be Access point, which 	*/
			/* then forward the frame to the appropriate destination.								*/
			pk_chstruct_ptr->rx_addr = ap_mac_address;
			}
		else
			{
			/* Otherwise set the final destination address.	*/				   
			pk_chstruct_ptr->rx_addr = destination_addr;
			}

		/* Source station address.	*/
		pk_chstruct_ptr->tx_addr = my_address;

		/* Setting the Rts frame type.	*/
		op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_CNTL_TYPE_FD, type, OPC_FIELD_SIZE_UNCHANGED);

		/* Setting the accept field to true, meaning the frame is a good frame.	*/
		op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_CNTL_ACCEPT_FD, OPC_TRUE, OPC_FIELD_SIZE_UNCHANGED);
				
		/* Setting the variable which keeps track of the last transmitted frame that needs response.	*/
		last_frametx_type = WlanC_Rts;
					
		/* Determining the size of the first data fragment or frame that need */
		/* to be transmitted following the RTS transmission.				  */				
		if (num_fragments > 1)
			{
			/* If there are more than one fragment to transmit then the */
			/* data segment of the first data frame will be the size of */
			/* fragmentation threshold. The total packet size will be   */
			/* data plus the overhead (which is 224 bits).				*/
			tx_datapacket_size = frag_threshold + WLANC_MPDU_HEADER_SIZE;
			}
		else
			/* If there is one data frame to transmit then the          */
			/* data segment of the first data frame will be the size of */
			/* the remainder computed earlier. The total packet size    */
			/* will be data plus the overhead (which is 224 bits).		*/
			{
			tx_datapacket_size = remainder_size + WLANC_MPDU_HEADER_SIZE;
			}

		/* Station is reserving channel bandwidth by using RTS frame, so    */
		/* in RTS the station will broadcast the duration it needs to send  */ 		 		
		/* one data frame and receive ACK for it. The total duration is the */
		/* the time required to transmit one data frame, plus one CTS frame */
		/* plus one ACK frame, and plus three SIFS intervals. While			*/
		/* computing the duration, call the two macros at different lines	*/
		/* to assure to use the correct value of the state variables within	*/
		/* the macros.														*/
		duration =  TXTIME_CTRL (WLANC_CTS_LENGTH) + TXTIME_CTRL (WLANC_ACK_LENGTH);
		duration += TXTIME_DATA (tx_datapacket_size) + 3 * sifs_time;            
		pk_chstruct_ptr->duration = duration;
				
		/* Setting RTS frame fields.										*/
		op_pk_fd_set_ptr (wlan_transmit_frame_ptr, WLANC_CNTL_HEADER_FD, pk_chstruct_ptr, OPC_FIELD_SIZE_UNCHANGED, 
					      wlan_pk_chstruct_copy, wlan_pk_chstruct_destroy, sizeof (WlanT_Control_Header_Fields));				
				
		/* Place the transmission data rate and physical layer technology	*/
		/* information into the packet.										*/
		wlan_frame_tx_phy_info_set (wlan_transmit_frame_ptr, tx_data_rate, phy_type, phy_char_flag);
	
		/* Adjust the packet size to accurately model the RTS message and	*/
		/* the PLCP overhead, which is physical layer technology dependent.	*/
		/* The default value for PLCP overhead is set for infra-red			*/
		/* technology.														*/
		if (phy_char_flag != WlanC_Infra_Red)
			total_frame_size = PLCP_OVERHEAD_CTRL (WLANC_RTS_LENGTH) * control_data_rate + WLANC_RTS_LENGTH;
		else
			total_frame_size = WLANC_DEFAULT_PLCP_OVERHEAD + WLANC_RTS_LENGTH;
		op_pk_total_size_set (wlan_transmit_frame_ptr, (OpT_Packet_Size) total_frame_size);
		
		/* Station update of its own nav_duration.							*/
		if (cfp_ap_medium_control == OPC_FALSE)
			nav_duration = current_time + duration + (double) (op_pk_total_size_get (wlan_transmit_frame_ptr)) / control_data_rate; 								 						
			
		/* CTS is expected in response to RTS.								*/						
		expected_frame_type = WlanC_Cts;

		/* Printing out information to ODB.									*/
		if (wlan_trace_active == OPC_TRUE)
			{
			sprintf (msg_string, "RTS is being transmitted for data packet " OPC_PACKET_ID_FMT ".", pkt_in_service);
			op_prg_odb_print_major (msg_string, OPC_NIL);
			}

		/* Update the control traffic sent statistics.						*/
		total_pk_size = (double) op_pk_total_size_get (wlan_transmit_frame_ptr);
		op_stat_write (ctrl_traffic_sent_handle_inbits, total_pk_size);
		op_stat_write (ctrl_traffic_sent_handle, 		1.0);

		/* Write a value of 0 for the end of transmission.					*/
		tx_end_time = current_time + total_pk_size / control_data_rate;
		op_stat_write_t (ctrl_traffic_sent_handle_inbits, 0.0, tx_end_time);
		op_stat_write_t (ctrl_traffic_sent_handle, 		  0.0, tx_end_time);
		}
	
	else if (frame_type == WlanC_Cts && fresp_to_send == WlanC_None)
		{
		/* Since we are sending this CTS message not a response, it is a	*/
		/* CTS-to-self message used by ERP STAs (11g stations).				*/
		
		/* Store the type of last transmission. IMPORTANT NOTE: In case of	*/
		/* CTS transmissions, the value of the state variable				*/
		/* last_frametx_type is set to WlanC_Cts ONLY for CTS-to-self		*/
		/* transmissions (i.e. it is not updated for regular CTS messages).	*/
		last_frametx_type = frame_type;
		
		/* Create a control message.										*/
		wlan_transmit_frame_ptr = op_pk_create_fmt ("wlan_control");

		/* Adjust the packet size if necessary to model the PLCP overhead	*/
		/* accurately, which is physical layer technology dependent. The	*/
		/* default value is set for infra-red technology.					*/
		if (phy_char_flag != WlanC_Infra_Red)
			{
			bulk_size = PLCP_OVERHEAD_CTRL (WLANC_CTS_LENGTH) * control_data_rate - WLANC_DEFAULT_PLCP_OVERHEAD;
			op_pk_bulk_size_set (wlan_transmit_frame_ptr, (OpT_Packet_Size) bulk_size);
			}
		
		/* Initializing RTS frame fields.									*/
		pk_chstruct_ptr = wlan_pk_chstruct_create ();

		/* Set the destination address to own address.						*/
		pk_chstruct_ptr->rx_addr = my_address;

		/* Determining the size of the first data fragment or frame that	*/
		/* need to be transmitted following the CTS-to-self transmission.	*/				
		if (num_fragments > 1)
			{
			/* If there are more than one fragment to transmit then the		*/
			/* data segment of the first data frame will be the size of 	*/
			/* fragmentation threshold. The total packet size will be data	*/
			/* plus the overhead (which is 224 bits).						*/
			tx_datapacket_size = frag_threshold + WLANC_MPDU_HEADER_SIZE;
			}
		else
			/* If there is one data frame to transmit then the data segment	*/
			/* of the first data frame will be the size of the remainder	*/
			/* computed earlier. The total packet size will be data plus	*/
			/* the overhead (which is 224 bits).							*/
			{
			tx_datapacket_size = remainder_size + WLANC_MPDU_HEADER_SIZE;
			}

		/* Compute the duration information that will be used by the		*/
		/* recipient MACs to update their NAVs. The duration must include a */
		/* SIFS time and the transmission time of the data frame that will	*/
		/* follow this control message. Additionally, another sifs time and	*/
		/* and an ACK transmission time must be included unless the data	*/
		/* packet has a broadcast address, which don't require an ACK.		*/
		if (destination_addr >= 0 || (bss_flag == OPC_TRUE && ap_flag == OPC_BOOLINT_DISABLED))
			{
			/* While computing the duration, call the two macros at			*/
			/* different lines to assure to use the correct value of the	*/
			/* state variables within the macros.							*/
			duration =  TXTIME_DATA (tx_datapacket_size);
			duration += TXTIME_CTRL (WLANC_ACK_LENGTH) + 2 * sifs_time;
			}
		else
			duration = TXTIME_DATA (tx_datapacket_size) + sifs_time; 
		pk_chstruct_ptr->duration = duration;
		
		/* Setting CTS frame type.											*/
		op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_CNTL_TYPE_FD, (int) WlanC_Cts, OPC_FIELD_SIZE_UNCHANGED);

		/* Initialize the "Accept" field.									*/
		op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_CNTL_ACCEPT_FD, OPC_TRUE, OPC_FIELD_SIZE_UNCHANGED);
				
		/* Setting CTS frame fields.										*/
		op_pk_fd_set_ptr (wlan_transmit_frame_ptr, WLANC_CNTL_HEADER_FD, pk_chstruct_ptr, OPC_FIELD_SIZE_UNCHANGED,
						  wlan_pk_chstruct_copy, wlan_pk_chstruct_destroy, sizeof (WlanT_Control_Header_Fields));
		
		/* We expect to receive our own CTS when sending CTS-to-self.		*/
		expected_frame_type = WlanC_Cts;	

		/* Update the control traffic sent statistics.						*/
		total_pk_size = (double) op_pk_total_size_get (wlan_transmit_frame_ptr);
		op_stat_write (ctrl_traffic_sent_handle_inbits, total_pk_size);
		op_stat_write (ctrl_traffic_sent_handle, 		1.0);

		/* Write a value of 0 for the end of transmission.					*/
		tx_delay    = total_pk_size / control_data_rate;
		tx_end_time = current_time + tx_delay;
		op_stat_write_t (ctrl_traffic_sent_handle_inbits, 0.0, tx_end_time);
		op_stat_write_t (ctrl_traffic_sent_handle, 		  0.0, tx_end_time);

		/* We need to update our own NAV.									*/
		if (cfp_ap_medium_control == OPC_FALSE)
			nav_duration = current_time + duration + tx_delay; 								 						

		/* Place the transmission data rate and physical layer technology	*/
		/* information into the packet.										*/
		wlan_frame_tx_phy_info_set (wlan_transmit_frame_ptr, tx_data_rate, phy_type, phy_char_flag);
		
		/* Send a copy of the packet to ourselves directly, since we will	*/
		/* not receive a transmission that is made by our own transmitter.	*/
		/* Add a very small delay to the transmission delay to guarantee	*/
		/* that we receive the copy a moment after our transmitter			*/
		/* completes our transmission.										*/
		op_pk_deliver_delayed (op_pk_copy (wlan_transmit_frame_ptr), my_objid, LOW_LAYER_INPUT_STREAM, (tx_delay + WLANC_CTS_TO_SELF_RX_DELAY));
		
		/* Printing out information to ODB.									*/
		if (wlan_trace_active == OPC_TRUE)
			{
			op_prg_odb_print_major ("CTS-to-self is being transmitted.", OPC_NIL);
			}
		}
		
	else if (frame_type == WlanC_Cts)
		{
		/* Preparing CTS frame in response to the received RTS frame.		*/
			
		if ((pcf_flag == OPC_BOOLINT_ENABLED) || (wlan_flags->pcf_active == OPC_TRUE))
			last_frametx_type = frame_type;
				
		/* Since an CTS is a control response frame, adjust its				*/
		/* transmission rate based on the data rate of the RTS frame we		*/
		/* are replying, if operating in an 11a or all-11g BSS. Otherwise	*/
		/* use 1 Mbps, the mandatory PHY rate of 802.11/11b.				*/
		if (phy_type == WlanC_11a_PHY || (phy_type == WlanC_11g_PHY && wlan_flags->non_erp_present == OPC_FALSE))
			tx_data_rate = rcvd_frame_drate;
		else
			tx_data_rate = control_data_rate;
		
		/* Creating CTS packet format type.									*/
		wlan_transmit_frame_ptr = op_pk_create_fmt ("wlan_control");

		/* Adjust the packet size if necessary to model the PLCP overhead	*/
		/* accurately, which is physical layer technology dependent. The	*/
		/* default value is set for infra-red technology.					*/
		if (phy_char_flag != WlanC_Infra_Red)
			{
			bulk_size = PLCP_OVERHEAD_CTRL (WLANC_CTS_LENGTH) * tx_data_rate - WLANC_DEFAULT_PLCP_OVERHEAD;
			op_pk_bulk_size_set (wlan_transmit_frame_ptr, (OpT_Packet_Size) bulk_size);
			}
		
		/* Initializing CTS frame fields.									*/
		pk_chstruct_ptr = wlan_pk_chstruct_create ();
		
		/* Type of frame.													*/
   		type = WlanC_Cts;

		/* Destination station address.										*/
		pk_chstruct_ptr->rx_addr = remote_sta_addr;
			
		/* Station is reserving channel bandwidth by using RTS frame, so    */
		/* in RTS the station will broadcast the duration it needs to send  */ 		 		
		/* one data frame and receive ACK for it. Just subtract the			*/
		/* transmission of the CTS frame from updated NAV. Already waited	*/
		/* SIFS is subtracted within "current_time".						*/
		duration = nav_duration - (TXTIME_CTRL_DR (WLANC_CTS_LENGTH, tx_data_rate) + current_time);
		pk_chstruct_ptr->duration = duration;

		/* Setting CTS frame type.											*/
		op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_CNTL_TYPE_FD, type, OPC_FIELD_SIZE_UNCHANGED);

		/* Initialize the "Accept" field.									*/
		op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_CNTL_ACCEPT_FD, OPC_TRUE, OPC_FIELD_SIZE_UNCHANGED);
				
		/* Setting CTS frame fields.										*/
		op_pk_fd_set_ptr (wlan_transmit_frame_ptr, WLANC_CNTL_HEADER_FD, pk_chstruct_ptr, OPC_FIELD_SIZE_UNCHANGED,
						  wlan_pk_chstruct_copy, wlan_pk_chstruct_destroy, sizeof (WlanT_Control_Header_Fields));

		/* Place the transmission data rate and physical layer technology	*/
		/* information into the packet.										*/
		wlan_frame_tx_phy_info_set (wlan_transmit_frame_ptr, tx_data_rate, phy_type, phy_char_flag);
	
		/* Once CTS is transmitted in response to RTS then set the frame	*/
		/* response indicator to none frame as the response is already		*/
		/* generated.														*/
		fresp_to_send = WlanC_None;										
			
		/* No frame is expected once CTS is transmitted.					*/
		expected_frame_type = WlanC_None;	

		/* Printing out information to ODB.									*/
		if (wlan_trace_active == OPC_TRUE)
			{
			op_prg_odb_print_major ("CTS is being transmitted in response to RTS.", OPC_NIL);
			}

		/* Update the control traffic sent statistics.						*/
		total_pk_size = (double) op_pk_total_size_get (wlan_transmit_frame_ptr);
		op_stat_write (ctrl_traffic_sent_handle_inbits, total_pk_size);
		op_stat_write (ctrl_traffic_sent_handle, 		1.0);

		/* Write a value of 0 for the end of transmission.					*/
		tx_end_time = current_time + total_pk_size / tx_data_rate;
		op_stat_write_t (ctrl_traffic_sent_handle_inbits, 0.0, tx_end_time);
		op_stat_write_t (ctrl_traffic_sent_handle, 		  0.0, tx_end_time);
		}
	
	else if (frame_type == WlanC_Beac)
		{
		last_frametx_type = frame_type;

		/* Create packet container for beacon body.							*/
		seg_pkptr = op_pk_create_fmt ("wlan_beacon_body");

		/* Initialize the bit count that will be added to the size of the	*/
		/* beacon body to represent the size of the optional beacon frame	*/
		/* body elements.													*/
		add_beacon_size = 0;
		
		/* Create beacon body.												*/
		pk_bbstruct_ptr = wlan_pk_bbstruct_create ();

		/* Timestamp should be set to reference 1st bit of timestamp in		*/
		/* message at antenna (11.1.2.1). To reduce processing, it is		*/
		/* currently set for first bit of MAC frame at antenna (assuming no	*/
		/* PHY delay).														*/
		pk_bbstruct_ptr->timestamp  = current_time;
		pk_bbstruct_ptr->beacon_intv = beacon_int;
		
		/* if no PCF, No beacon starts a CFP.								*/ 
		if (pcf_flag == OPC_BOOLINT_DISABLED || poll_list_size == 0)           
			{
			pk_bbstruct_ptr->cf_par.cfp_count       = 1;   
			pk_bbstruct_ptr->cf_par.cfp_period      = 0;    
			}
		else											
			/* PCF implemented.												*/
			{
			/* When cfp_count is computed as "0" then this beacon			*/
			/* advertises the start of a CFP. Subtract one while finding	*/
			/* out the transmission number of this beacon, since the first	*/
			/* beacon is sent at "beacon_int" seconds instead of 0 seconds.	*/
			pk_bbstruct_ptr->cf_par.cfp_count = 
				(((int) ((current_time + PRECISION_RECOVERY) / beacon_int) - 1 - cfp_offset) % cfp_prd);    	
			
			/* Set the flag if this beacon will initiate a contention free	*/
			/* period.														*/
			if (pk_bbstruct_ptr->cf_par.cfp_count == 0)
				wlan_flags->pcf_active = OPC_TRUE;

			/* Set CFP period.												*/
			pk_bbstruct_ptr->cf_par.cfp_period = cfp_prd;   	
			
			/* Set CFP maximum duration.									*/
			pk_bbstruct_ptr->cf_par.cfp_maxduration = cfp_length;	

			/* If beginning a CFP.											*/
			if (pk_bbstruct_ptr->cf_par.cfp_count == 0)           	
				{ 							
				/* Find time remaining in current CFP.						*/
				pk_bbstruct_ptr->cf_par.cfp_durremaining = 
					cfp_length - (current_time - ((int) (current_time / beacon_int)) * beacon_int); 
				}
			
			/* Add the size of "CF Parameter Set" element to the beacon		*/
			/* size, which is 8 bytes.										*/
			add_beacon_size += WLANC_BC_COMP_CF_PS_SIZE;
			}
		
		/* If we are an 11g supporting AP, then set the non_erp_present bit	*/
		/* of the beacon if there are some non-ERP STAs in our BSS.			*/
		if (phy_type == WlanC_11g_PHY)
			{
			/* Lock the related mutex before checking the current number of	*/
			/* non-ERP STAs in our BSS.										*/
			op_prg_mt_mutex_lock (mapping_info_mutex, OPC_MT_MUTEX_LOCK_READER);
			
			/* Check whether there is a change in the count of non-ERP STAs	*/
			/* in our BSS.													*/
			if (wlan_flags->non_erp_present == OPC_FALSE && my_bss_info_ptr->non_erp_sta_count > 0)
				{
				/* We have a new non-ERP STA in our BSS. Set the flag.		*/
				wlan_flags->non_erp_present = OPC_TRUE;
				
				/* Increase the slot time to 20 usec and recompute the		*/
				/* dependent parameters.									*/
				wlan_slot_time_set (20E-06);

				/* Reduce the control frame data rate to 802.11/11b			*/
				/* mandatory data rate.										*/
				control_data_rate = WLANC_11b_MIN_MANDATORY_DRATE;
				}
			
			else if (wlan_flags->non_erp_present == OPC_TRUE && my_bss_info_ptr->non_erp_sta_count == 0)
				{
				/* All the non-ERP STAs have left our BSS. Reset the flag.	*/
				wlan_flags->non_erp_present = OPC_FALSE;
				
				/* Decrease the slot time to 9 usec and recompute the		*/
				/* dependent parameters.									*/
				wlan_slot_time_set (9E-06);
				
				/* Set our data transmission rate to the original data rate,*/
				/* since we could be using a lower data rate to communicate	*/
				/* with non-ERP STAs.										*/
				operational_speed = data_tx_rate;

				/* Reselect the control frame data rate. Choose the highest	*/
				/* mandatory data rate that is equal to or lower than the	*/
				/* data rate specified for data transmissions.				*/
				for (i = 0; data_tx_rate < WLANC_11g_MANDATORY_DRATE_ARRAY [i]; i++);
				control_data_rate = WLANC_11g_MANDATORY_DRATE_ARRAY [i];
				}
			
			/* Unlock the mutex since we are done accessing the BSS info.	*/
			op_prg_mt_mutex_unlock (mapping_info_mutex);
			
			/* Set the non_erp_present bit value of the beacon frame.		*/
			pk_bbstruct_ptr->non_erp_present = wlan_flags->non_erp_present;

			/* Since we are an AP supporting 11g data rates, make sure that	*/
			/* we transmit our beacon messages with the lowest 802.11/11b	*/
			/* mandatory data rate so that if there are any roaming non-ERP	*/
			/* STAs in the network that are in the scanning process, they	*/
			/* can decode our beacons and join into our BSS.				*/
			tx_data_rate = WLANC_11b_MIN_MANDATORY_DRATE;
			
			/* Make additions to the size of beacon due to the 11g specific	*/
			/* elements added to the beacon frame body. 11g APs are assumed	*/
			/* to support 12 data rates. Hence, increase the size of the	*/
			/* "Supported Rates" by six rates (= 6 bytes) and also add an	*/
			/* "Extended Supported Rates" elements for the remaining 4		*/
			/* rates, which also becomes 6 bytes. Finally add the sizes of	*/
			/* "DS Parameter Set" and "ERP Information" elements, which are	*/
			/* both 3 bytes.												*/
			add_beacon_size += 10 * WLANC_BC_COMP_SUP_RATE_SIZE + WLANC_BC_COMP_EXT_RATES_SIZE + 
							   WLANC_BC_COMP_DS_PS_SIZE + WLANC_BC_COMP_ERP_INFO_SIZE;
			}
		
		else if (phy_type == WlanC_11a_PHY)
			{
			/* 11a-APs support 8 data rates. Adjust the beacon body size	*/
			/* for additional 6 rates in the "Supported Rates" element.		*/
			add_beacon_size += 6 * WLANC_BC_COMP_SUP_RATE_SIZE;
			}
		else if (phy_char_flag == WlanC_Direct_Sequence)
			{
			/* DSSS-APs support 4 data rates. Adjust the beacon body size	*/
			/* for additional 2 rates in the "Supported Rates" element, and	*/
			/* for the "DS Parameter Set" element.							*/
			add_beacon_size += 2 * WLANC_BC_COMP_SUP_RATE_SIZE + WLANC_BC_COMP_DS_PS_SIZE;
			}
		else if (phy_char_flag == WlanC_Frequency_Hopping)
			{
			/* Add 7 bytes to the beacon size to represent the "FH			*/
			/* Parameter Set" element, which exists in beacons generated	*/
			/* by the APs using frequency-hopping PHYs.						*/
			add_beacon_size += WLANC_BC_COMP_FH_PS_SIZE;
			}			
		
		/* If any, add the bits of optional beacon frame body elements to	*/
		/* the size of the beacon as bulk size.								*/
		if (add_beacon_size > 0)
			op_pk_bulk_size_set (seg_pkptr, (OpT_Packet_Size) add_beacon_size);
		
		/* Use data frame format for beacon frame since we need frame body.	*/

		/* Creating transmit data packet type.								*/
		wlan_transmit_frame_ptr = op_pk_create_fmt ("wlan_mac");
		
		/* Set destination address to broadcast since unicast not supported.*/
		destination_addr = MAC_BROADCAST_ADDR;

		/* Prepare data frame fields for transmission.	*/		
		pk_dhstruct_ptr = wlan_pk_dhstruct_create ();

		pk_dhstruct_ptr->retry = OPC_FALSE;				
		pk_dhstruct_ptr->order = 1;
		pk_dhstruct_ptr->sequence_control = packet_seq_control;

		/* During CFP the duration field should read 32768. (Section 7.1.3.2 of spec) */
		/* During CP should read zero since broadcast (Section 7.2.3)  */
		if (pk_bbstruct_ptr->cf_par.cfp_count == 0)	
			duration = 32768.0;
		else 
			duration = 0.0;
				
		/* Setting the Header field structure.	*/
		pk_dhstruct_ptr->duration  = duration;
		pk_dhstruct_ptr->address1  = destination_addr; /* Always set for broadcast for now */
		pk_dhstruct_ptr->address2  = my_address;
		
		/* This value is checked at the receiving end to see if this frame was intended for this bss id */
		pk_dhstruct_ptr->address3  = bss_id;

		/* Management frames (Beacon) never involve DS.	*/
		pk_dhstruct_ptr->fromds	 = 0;
		pk_dhstruct_ptr->tods    = 0;
	
		/* Start setting the packet fields.								*/
		op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_DATA_TYPE_FD, frame_type, OPC_FIELD_SIZE_UNCHANGED);
		op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_DATA_ACCEPT_FD, OPC_TRUE, OPC_FIELD_SIZE_UNCHANGED);

		/* Set the frame control field.									*/
		op_pk_fd_set_ptr (wlan_transmit_frame_ptr, WLANC_DATA_HEADER_FD, pk_dhstruct_ptr, OPC_FIELD_SIZE_UNCHANGED,	
					      wlan_pk_dhstruct_copy, wlan_pk_dhstruct_destroy, sizeof (WlanT_Data_Header_Fields));

		/* If this is the start of the CFP, reset the NAV 				*/
		/* Any frame sequences in progress will be interrupted anyway.	*/
		if(pk_bbstruct_ptr->cf_par.cfp_count == 0)
			nav_duration = current_time;
		
		/* The beacon body is placed in the Packet container.			*/
		op_pk_fd_set_ptr (seg_pkptr, WLANC_BEACON_BODY_FD, pk_bbstruct_ptr, OPC_FIELD_SIZE_UNCHANGED,	
					      wlan_pk_bbstruct_copy, wlan_pk_bbstruct_destroy, sizeof (WlanT_Beacon_Body_Fields));

		/* The beacon body "packet" is placed in the Frame Body field	*/
		op_pk_fd_set_pkt (wlan_transmit_frame_ptr, WLANC_DATA_BODY_FD, seg_pkptr, OPC_FIELD_SIZE_UNCHANGED);
	
		/* Place the transmission data rate and physical layer			*/
		/* technology information into the packet.						*/
		wlan_frame_tx_phy_info_set (wlan_transmit_frame_ptr, tx_data_rate, phy_type, phy_char_flag);
	
		/* Adjust the packet size if necessary to model the PLCP		*/
		/* overhead accurately, which is physical layer technology		*/
		/* dependent. The default value is set for infra-red technology.*/
		if (phy_char_flag != WlanC_Infra_Red)
			{
			total_plcp_overhead = PLCP_OVERHEAD_CTRL_DR (((int) op_pk_total_size_get (wlan_transmit_frame_ptr) -  WLANC_DEFAULT_PLCP_OVERHEAD), tx_data_rate);
			bulk_size = total_plcp_overhead * tx_data_rate - WLANC_DEFAULT_PLCP_OVERHEAD;
			op_pk_bulk_size_set (wlan_transmit_frame_ptr, (OpT_Packet_Size) bulk_size);
			}
		
		/* Clear expected frame time since any existing valid frame		*/
		/* sequences have been interrupted anyway.						*/
		expected_frame_type = WlanC_None;
		
		/* Clear tx beacon flag.										*/
		wlan_flags->tx_beacon = OPC_FALSE;		

		/* Update the management traffic sent statistics.				*/
		total_pk_size = (double) op_pk_total_size_get (wlan_transmit_frame_ptr);
		op_stat_write (mgmt_traffic_sent_handle_inbits, total_pk_size);
		op_stat_write (mgmt_traffic_sent_handle, 		1.0);

		/* Write a value of 0 for the end of transmission.				*/
		tx_end_time = current_time + total_pk_size / tx_data_rate;
		op_stat_write_t (mgmt_traffic_sent_handle_inbits, 0.0, tx_end_time);
		op_stat_write_t (mgmt_traffic_sent_handle, 		  0.0, tx_end_time);

		/* Printing out information to ODB.								*/
		if (wlan_trace_active == OPC_TRUE)
			{
			op_prg_odb_print_major ("Beacon is being transmitted by the Access Point.", OPC_NIL);
			}
		}

	/* Check if the frame type to be transmitted is a data null/cf ack/cf poll */
	else if ((frame_type == WlanC_Data_Null)|| (frame_type == WlanC_Cf_Ack)	|| 
			(frame_type == WlanC_Cf_Poll)	|| (frame_type == WlanC_Cf_A_P))
		{			
		/* Preserve the frame being transmitted */
		last_frametx_type = frame_type;
		
		/* Adjust the transmission data rate based on the operational speed.*/
		tx_data_rate = operational_speed;

		/* Set active poll flag if this is a poll frame */
		if ((frame_type == WlanC_Cf_Poll) || (frame_type == WlanC_Cf_A_P))
			{
			wlan_flags->active_poll = OPC_TRUE;
			}
		
		/* If it is a retransmission of a packet then no need 	*/
     	/* of preparing data frame.				*/
		if	(((frame_type == WlanC_Data_Null) || (frame_type == WlanC_Cf_Ack)) || 
			((pcf_retry_count == 0) && (poll_fail_count == 0)))
			{
			/* Creating transmit data packet type.							*/
			wlan_transmit_frame_ptr = op_pk_create_fmt ("wlan_mac");
				
			/* Prepare data frame fields for transmission.					*/		
			pk_dhstruct_ptr = wlan_pk_dhstruct_create ();
			pk_dhstruct_ptr->retry = OPC_FALSE;				
			pk_dhstruct_ptr->order = 1;
			pk_dhstruct_ptr->sequence_control = packet_seq_control;
			pk_dhstruct_ptr->more_frag = 0;

			/* Set duration field */
			/* During PCF the duration field should read 32768. (Section 7.1.3.2 of spec) */
			duration = 32768.0;

			/* Setting the Header field structure.	*/
			pk_dhstruct_ptr->duration  = duration;
			pk_dhstruct_ptr->address1  = destination_addr;
			pk_dhstruct_ptr->address2  = my_address;

			/* In the BSS network the Data frame is going from AP to sta then fromds bit is set and	*/
 			/* address3 contains the address of the MAC that is the original sender of the packet.	*/
			if (ap_flag == OPC_BOOLINT_ENABLED)
				{
				pk_dhstruct_ptr->fromds = 1;
				pk_dhstruct_ptr->address3 = orig_source_addr;
				}
			else
				{
				pk_dhstruct_ptr->fromds = 0;
				}

			/* if in the BSS network the Data frame is going from sta to AP then tods bit is set.	*/					
    		if ((bss_flag == OPC_TRUE) && (ap_flag == OPC_BOOLINT_DISABLED))
				{
				pk_dhstruct_ptr->tods = 1;

				/* If Infrastructure BSS then the immediate destination will be Access point, which 	*/
				/* then forward the frame to the appropriate destination.								*/
				pk_dhstruct_ptr->address1 = ap_mac_address;
				pk_dhstruct_ptr->address3 = destination_addr;
				}
			else
				{
				pk_dhstruct_ptr->tods = 0;
				}
	
			op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_DATA_TYPE_FD,   frame_type,         OPC_FIELD_SIZE_UNCHANGED);
			op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_DATA_ACCEPT_FD, OPC_TRUE,           OPC_FIELD_SIZE_UNCHANGED);
			op_pk_fd_set_pkid  (wlan_transmit_frame_ptr, WLANC_DATA_PKID_FD,   pcf_pkt_in_service, OPC_FIELD_SIZE_UNCHANGED);
				
			/* Set the frame control field.				*/
			op_pk_fd_set_ptr (wlan_transmit_frame_ptr, WLANC_DATA_HEADER_FD, pk_dhstruct_ptr, OPC_FIELD_SIZE_UNCHANGED,	
			                  wlan_pk_dhstruct_copy, wlan_pk_dhstruct_destroy, sizeof (WlanT_Data_Header_Fields));

			/* Need to create dummy "Frame Body" so use beacon frame and	*/
			/* set size to zero. Create packet container for beacon body.	*/
			seg_pkptr = op_pk_create_fmt ("wlan_beacon_body");
			op_pk_total_size_set (seg_pkptr, (OpT_Packet_Size) (0));

			/* The actual data is placed in the Frame Body field.			*/
			op_pk_fd_set_pkt (wlan_transmit_frame_ptr, WLANC_DATA_BODY_FD, seg_pkptr, OPC_FIELD_SIZE_UNCHANGED);

			/* If enabled, print out an ODB trace message.					*/
			if (wlan_trace_active == OPC_TRUE)
				{
				if (frame_type == WlanC_Cf_Ack)
					op_prg_odb_print_major ("A CF-ACK message is being transmitted in reply to received poll+data message.", OPC_NIL);
				else if (frame_type == WlanC_Data_Null)
					op_prg_odb_print_major ("A Null (no data) message is being transmitted in reply to received poll message.", OPC_NIL);
				else
					{
					wlan_frame_type_conv (frame_type, frame_type_str);
					sprintf (msg_string, " A %s message is being transmitted by the Access Point.", frame_type_str);
					op_prg_odb_print_major (msg_string, OPC_NIL);
					}
				}
			
			/* Add some bulk to the packet to model the transmission delay	*/
			/* of PLCP fields accurately which are always transmitted at	*/
			/* 1 Mbps regardless of the actual data rate used for data		*/
			/* frames.														*/
			total_plcp_overhead = PLCP_OVERHEAD_DATA ((int) op_pk_total_size_get (wlan_transmit_frame_ptr) - WLANC_DEFAULT_PLCP_OVERHEAD);
			bulk_size = total_plcp_overhead * operational_speed - WLANC_DEFAULT_PLCP_OVERHEAD;
			op_pk_bulk_size_set (wlan_transmit_frame_ptr, (OpT_Packet_Size) bulk_size);
		
			/* Make copy of the frame before transmission	*/
			if ((frame_type != WlanC_Data_Null) && (frame_type != WlanC_Cf_Ack))
				{
				wlan_pcf_transmit_frame_copy_ptr = op_pk_copy (wlan_transmit_frame_ptr);
				}
			}
		else 
			{
			/* If it is a retransmission then just transmit the previous frame	*/			
			wlan_transmit_frame_ptr = op_pk_copy (wlan_pcf_transmit_frame_copy_ptr);

			/* If retry count is non-zero means that the frame is a 	*/
			/* retransmission of the last transmitted frame.			*/
			op_pk_fd_access_ptr (wlan_transmit_frame_ptr, WLANC_DATA_HEADER_FD, (void **) &pk_dhstruct_ptr);
			pk_dhstruct_ptr->retry = OPC_TRUE;

			/* Read back duration field for debug stuff.				*/
			duration =	pk_dhstruct_ptr->duration;
			
			/* Re-write the packet type since the poll message sent		*/
			/* earlier may have a piggy-backed ACK, which will not be	*/
			/* repeated in this retransmission.							*/
			op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_DATA_TYPE_FD, frame_type, OPC_FIELD_SIZE_UNCHANGED); 
			
			/* If enabled, print out an ODB trace message.				*/
			if (wlan_trace_active == OPC_TRUE)
				{
				wlan_frame_type_conv (frame_type, frame_type_str);
				sprintf (msg_string, "The Access Point is retransmitting a %s message.", frame_type_str);							
				op_prg_odb_print_major (msg_string, OPC_NIL);
				}
			}

		/* Place the transmission data rate and physical layer			*/
		/* technology information into the packet.						*/
		wlan_frame_tx_phy_info_set (wlan_transmit_frame_ptr, tx_data_rate, phy_type, phy_char_flag);
	
		/* Update the data traffic sent statistics.						*/
		total_pk_size = (double) op_pk_total_size_get (wlan_transmit_frame_ptr);
		op_stat_write (data_traffic_sent_handle_inbits, total_pk_size);
		op_stat_write (data_traffic_sent_handle, 1.0);

		/* Write a value of 0 for the end of transmission.				*/
		tx_end_time = current_time + total_pk_size / operational_speed;
		op_stat_write_t (data_traffic_sent_handle_inbits, 0.0, tx_end_time);
		op_stat_write_t (data_traffic_sent_handle, 0.0, tx_end_time);

		/* No ACK expected for non-data poll frames but do expect some	*/
		/* type of Data frame in response.								*/
		if	((frame_type == WlanC_Cf_Poll)	|| (frame_type == WlanC_Cf_A_P))
			expected_frame_type = WlanC_Data;
		else
			expected_frame_type = WlanC_None;

		/* Once Ack is transmitted in response to Data frame then set	*/
		/* the frame response indicator to none frame as the response	*/
		/* is already generated.										*/
		fresp_to_send = WlanC_None;					
		}

	/* Preparing Contention Free end frame if no more stations */
	/* to poll or Cfp_End interrupt.						   */
	else if	((frame_type == WlanC_Cf_End) || (frame_type == WlanC_Cf_End_A))
		{
	   	if ((pcf_flag == OPC_BOOLINT_ENABLED) || (wlan_flags->pcf_active == OPC_TRUE))
			last_frametx_type = frame_type;
			
		/* Creating Cf_End packet format type.								*/
		wlan_transmit_frame_ptr = op_pk_create_fmt ("wlan_control");

		/* Setting ack frame fields.										*/
		pk_chstruct_ptr = wlan_pk_chstruct_create ();

		/* Set duration field */
		/* CF_End duration should always read zero.(Section 7.2.1.6 of spec)*/
		duration = 0;
		pk_chstruct_ptr->duration = duration;

		/* CF End is a broadcast, so set destination address to -1.			*/
		pk_chstruct_ptr->rx_addr = MAC_BROADCAST_ADDR;

		/* The tx address conveys our own BSS ID in the CF-End messages.	*/
		pk_chstruct_ptr->tx_addr = bss_id;

		/* Setting frame type.												*/
		op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_CNTL_TYPE_FD, frame_type, OPC_FIELD_SIZE_UNCHANGED);
			
		/* Setting the accept field to true, meaning the frame is a good	*/
		/* frame.															*/
		op_pk_fd_set_int32 (wlan_transmit_frame_ptr, WLANC_CNTL_ACCEPT_FD, OPC_TRUE, OPC_FIELD_SIZE_UNCHANGED);

		op_pk_fd_set_ptr (wlan_transmit_frame_ptr, WLANC_CNTL_HEADER_FD, pk_chstruct_ptr, OPC_FIELD_SIZE_UNCHANGED,
						  wlan_pk_chstruct_copy, wlan_pk_chstruct_destroy, sizeof (WlanT_Control_Header_Fields));

		/* Place the transmission data rate and physical layer technology	*/
		/* information into the packet.										*/
		wlan_frame_tx_phy_info_set (wlan_transmit_frame_ptr, tx_data_rate, phy_type, phy_char_flag);
	
		/* Adjust the packet size to model the Cf_End message and the PLCP	*/
		/* overhead, which is physical layer technology dependent,			*/
		/* accurately. The default value for PLCP overhead is set for		*/
		/* infra-red technology. Also note that the size of CF End message	*/
		/* is equal to the size of the RTS message.							*/
		if (phy_char_flag != WlanC_Infra_Red)
			total_frame_size = PLCP_OVERHEAD_CTRL (WLANC_RTS_LENGTH) * control_data_rate + WLANC_RTS_LENGTH;
		else
			total_frame_size = WLANC_DEFAULT_PLCP_OVERHEAD + WLANC_RTS_LENGTH;
		op_pk_total_size_set (wlan_transmit_frame_ptr, (OpT_Packet_Size) total_frame_size);

		/* Since no frame is expected, the expected frame type field */
		/* to nil.                                                   */
		expected_frame_type = WlanC_None;	
	
		/* No response is expected so set indicator accordingly		*/
		fresp_to_send = WlanC_None;			

		/* Printing out information to ODB.	*/
		if (wlan_trace_active == OPC_TRUE)
			{
			if (frame_type == WlanC_Cf_End_A)
				op_prg_odb_print_major ("A CF-End+ACK is being transmitted by the Access Point.", OPC_NIL);
			else
				op_prg_odb_print_major ("A CF-End is being transmitted by the Access Point.", OPC_NIL);
			}

		/* Since CFP over, clean up indicators */
		if ((op_ev_valid (cfp_end_evh) == OPC_TRUE) && (wlan_flags->tx_cf_end == OPC_FALSE))
			{op_ev_cancel (cfp_end_evh);}

		/* Check if a PCF beacon has been overrun before clearing pcf_active flag */
		if ((wlan_flags->tx_beacon == OPC_TRUE) && 
			((((int) ((current_time + PRECISION_RECOVERY) / beacon_int) - 1 - cfp_offset) % cfp_prd) == 0))
			{
			/* PCF beacon has been overrun so don't clear flag */
			}
		else 
			{
			wlan_flags->pcf_active = OPC_FALSE;
			}

		wlan_flags->tx_cf_end			= OPC_FALSE;
		wlan_flags->pcf_side_traf		= OPC_FALSE;
		wlan_flags->active_poll			= OPC_FALSE;
		wlan_flags->more_data			= OPC_FALSE;
		poll_fail_count = 0;

		/* Update the control traffic sent statistics.					*/
		total_pk_size = (double) op_pk_total_size_get (wlan_transmit_frame_ptr);
		op_stat_write (ctrl_traffic_sent_handle_inbits, total_pk_size);
		op_stat_write (ctrl_traffic_sent_handle, 		1.0);

		/* Write a value of 0 for the end of transmission.				*/
		tx_end_time = current_time + total_pk_size / control_data_rate;
		op_stat_write_t (ctrl_traffic_sent_handle_inbits, 0.0, tx_end_time);
		op_stat_write_t (ctrl_traffic_sent_handle, 		  0.0, tx_end_time);
		}
	else
		{
		wlan_error_print ("Transmission request for unexpected frame type.", OPC_NIL, OPC_NIL);
		}
			
	/* Send packet to the transmitter.										*/
	op_pk_send (wlan_transmit_frame_ptr, LOW_LAYER_OUTPUT_STREAM);	
	wlan_flags->transmitter_busy = OPC_TRUE;
	
	/* Set the flag if the current transmission requires signal extension.	*/
	if (phy_char_flag == WlanC_ERP_OFDM_11g && tx_data_rate > 5500000.0 && tx_data_rate != 11000000.0)
		wlan_flags->wait_signal_ext = OPC_TRUE;

	/* Clear ignore busy flag in case it was set.							*/
	wlan_flags->ignore_busy = OPC_FALSE;
	
	/* Clear PCF side traffic flag in case it was set.						*/
	wlan_flags->pcf_side_traf = OPC_FALSE;
	
	/* Clear polled flag in case it was set.								*/
	wlan_flags->polled = OPC_FALSE;
	
	FOUT;
	}

static void
wlan_slot_time_set (double new_slot_time)
 	{
	/** This function sets the slot time to the given value and recomputed	**/
	/** the values of the MAC parameters that depend on the value of the	**/
	/** slot time.															**/
	FIN (wlan_slot_time_set (new_slot_time));
	
	/* Update the slot time with the new value.								*/
	slot_time = new_slot_time;
	
	/* Recompute the inter-frame spacing values that depend on slot time.	*/
	difs_time = sifs_time + 2 * slot_time;
	pifs_time = sifs_time + slot_time;
	eifs_time = difs_time + sifs_time + TXTIME_CTRL_DR (WLANC_ACK_LENGTH, WLANC_11b_MIN_MANDATORY_DRATE);
	
	FOUT;
	}

static void 
wlan_interrupts_process (void)
	{
	int					integer_sta_mac_addr;
	OpT_Int64			sta_mac_addr;
	WlanT_Peer_Info*	peer_info_ptr;
	void*				dummy_ptr;

	/** This routine handles the appropriate processing need for each type 	**/
	/** of remote interrupt. The type of interrupts are: stream interrupts	**/
	/** (from lower and higher layers), stat interrupts (from receiver and 	**/
	/** transmitter).                                                      	**/
	FIN (wlan_interrupts_process (void));

	/* Check if debugging is enabled.										*/
	wlan_trace_active = (debug_mode && op_prg_odb_ltrace_active ("wlan"));

	/* Determine the current simulation time.								*/	
	current_time = op_sim_time ();
	
	/* Determine interrupt type and code to divide treatment along the		*/
	/* lines of interrupt type.						  						*/
	intrpt_type = op_intrpt_type ();
	intrpt_code = (WlanT_Mac_Intrpt_Code) op_intrpt_code ();

	/* Stream interrupts are either arrivals from the higher layer, or		*/
	/* from the physical layer.												*/
	if (intrpt_type == OPC_INTRPT_STRM)
		{
		/* Determine the stream on which the arrival occurred.				*/
		i_strm = op_intrpt_strm ();

		/* If the event arrived from higher layer then queue the packet	and	*/
		/* the destination address.											*/
		if (i_strm == instrm_from_mac_if)
			{
			/* Process stream interrupt received from higher layer.			*/
			wlan_higher_layer_data_arrival ();
			}

		/* If the event was an arrival from the physical layer, accept the	*/
		/* packet and decapsulate it.								 		*/
		else 
			{
			/* We need to update the receiver_busy indicator first since it */
			/* will be used with wlan_physical_layer_data_arrival ().		*/			
			
			/* Check the receiver's status and update the related flag if	*/
			/* necessary. The statwire interrupt from the receiver won't	*/
			/* reset this flag when signal extension is used in 11g			*/
			/* networks.													*/
			if (wlan_flags->receiver_busy == OPC_TRUE && rx_state_info_ptr->rx_end_time - PRECISION_RECOVERY <= current_time)
				{		
				wlan_flags->receiver_busy = OPC_FALSE;
				wlan_flags->collision     = OPC_FALSE;
				}
			
			/* Process stream interrupt received from physical layer.		*/			 
			wlan_physical_layer_data_arrival ();
   			}
	 	}	

	/* Handle stat interrupt received from the receiver.					*/
	else if (intrpt_type == OPC_INTRPT_STAT)
		{
		/* Make sure it is not a stat interrupt from the transmitter.		*/
		if (intrpt_code < TRANSMITTER_BUSY_INSTAT)
			{
			/* One of receiver channels is changing its status. Update the	*/
			/* channel status flag.											*/
			wlan_mac_rcv_channel_status_update (intrpt_code);
			}
		else
			{
			/* Reset bad_packet_dropped flag upon a stat interrupt from the	*/
			/* transmitter.													*/
			wlan_flags->bad_packet_dropped = OPC_FALSE;
			}
		}

	/* Handle interrupt from beacon timer.									*/
	else if (intrpt_type == OPC_INTRPT_SELF)
		{
		if (intrpt_code == WlanC_Beacon_Tx_Time)
			{
			/* If AP and time to transmit beacon then set the flag.			*/
			if (ap_flag == OPC_BOOLINT_ENABLED) 
				{
				wlan_flags->tx_beacon = OPC_TRUE;

				/* If we are sending certain number of beacons, rather than	*/
				/* periodically for whole simulation, then decrement the	*/
				/* number of needed beacon transmissions.					*/
				if (BEACON_TX_EFFICIENCY_ENABLED)
					rem_beacon_tx--;
				
				/* Set timer for next beacon transmission if we are sending	*/
				/* periodic beacons, or if we have more beacons to send		*/
				/* even if the beacon efficiency mode is enabled. 			*/
				/* Important: the check must be	"!= 0" rather than "> 0",	*/
				/* because the count is -1 when this MAC sends periodic		*/
				/* beacons.													*/
				if (rem_beacon_tx != 0)
					{
					beacon_tx_time += beacon_int;
					op_intrpt_schedule_self (beacon_tx_time , WlanC_Beacon_Tx_Time);
					}
				}
		
			/* Make the initializations for CFP if the BSS has at least one	*/
			/* PCF enabled station and if we are not already in CFP.		*/
			if (active_pc == OPC_TRUE && poll_list_size > 0 && wlan_flags->pcf_active == OPC_FALSE)
				{
				if ((((int) ((current_time + PRECISION_RECOVERY) / beacon_int) - 1 - cfp_offset) % cfp_prd) == 0)
					{
					/* If this is an AP.									*/
					if (ap_flag == OPC_BOOLINT_ENABLED)
						{
						cfp_end_evh = op_intrpt_schedule_self (current_time + cfp_length , WlanC_Cfp_End);

						/* For current polling implementation, the			*/
						/* fragmentation buffer is assumed empty at the		*/
						/* start of the CFP, and polling always starts with	*/
						/* the first station. This variable will be			*/
						/* incremented for first station.  					*/
						poll_index = -1;

						/* Reset failed poll counter.						*/
						poll_fail_count = 0;

						/* Reset poll for more data flag.					*/
						wlan_flags->more_data = OPC_FALSE;
					
						/* Reset pcf queue offset.							*/
						pcf_queue_offset = 0;
						}
					else 
						{
						/* Update the NAV duration for a non-AP station.	*/
						if (nav_duration < (cfp_length + current_time))
							{
							nav_duration = cfp_length + current_time;
							}
						}
					}
				} 
			}
	
		/* Handle interrupt from CFP end timer.								*/
		else if (intrpt_code == WlanC_Cfp_End)
			{
			/* Set Transmit Contention Free (CF) Period End flag.			*/
			wlan_flags->tx_cf_end = OPC_TRUE;
		
			/* Don't clear CFP Flag till after transmitting CF End. Don't	*/
			/* touch NAV - It should clear itself anyway.					*/
			}

		else if (intrpt_code == WlanC_NAV_Reset_Time)
			{
			/* Reset NAV by setting nav_duration to current time minus DIFS	*/
			/* to be fully synchronized with other STAs in the LAN.			*/
			nav_duration = current_time - difs_time;
	
			/* If we were deferring for the end of channel's reservation,	*/
			/* cancel the end of deference interrupt since with this		*/
			/* interrupt we will be leaving the DEFER state.				*/
			if (op_ev_valid (deference_evh))
				op_ev_cancel (deference_evh);		
			}
		}

	else if (intrpt_type == OPC_INTRPT_REMOTE && intrpt_code == WlanC_Beacon_Tx_Time)
		{
		/* If we enter here then it means, although we don't send periodic	*/
		/* beacons due to beacon efficiency mode, we have been requested to	*/
		/* send certain number of beacons to report a change in the BSS		*/
		/* configuration. Store that number.								*/ 
		rem_beacon_tx = beacon_tx_count;
		
		/* Set the beacon transmission flag, if it has been at least "a		*/
		/* beacon interval" since the last beacon.							*/
		if (current_time - beacon_tx_time >= beacon_int)
			{
			wlan_flags->tx_beacon = OPC_TRUE;
			rem_beacon_tx--;
			beacon_tx_time = current_time;
			}
		
		/* Schedule the transmission of the next beacon if the				*/
		/* configuration requires that we need to send more than one beacon	*/
		/* to report such changes, unless there is already a scheduled one.	*/
		if (rem_beacon_tx != 0 && beacon_tx_time <= current_time)
			{
			beacon_tx_time += beacon_int;
			op_intrpt_schedule_self (beacon_tx_time, WlanC_Beacon_Tx_Time);
			}
		}
	
	else if (intrpt_type == OPC_INTRPT_PROCESS)
		{
		/* This is an interrupt scheduled by a roaming STA to report its	*/
		/* association or disassociation. Find out the address of the STA.	*/
		integer_sta_mac_addr = intrpt_code >> WLANC_ADDRESS_BIT_SHIFT;
		sta_mac_addr = integer_sta_mac_addr;
		
		/* Is it an association or disassociation?							*/
		if (intrpt_code & WLANC_ASSOCIATION_BIT)
			{
			/* Create an information record for this STA joining our BSS	*/
			/* and initialize the last received frame seqeunce ID			*/
			/* information, which will be used for duplicate detection.		*/
			peer_info_ptr = (WlanT_Peer_Info *) op_prg_mem_alloc (sizeof (WlanT_Peer_Info));
			peer_info_ptr->seq_cntl = 0xFFFF;
			
			/* Store the ERP capability (11g support) information.			*/
			peer_info_ptr->is_erp = (intrpt_code & WLANC_ERP_SUPPORT_BIT) ? OPC_TRUE : OPC_FALSE;
			
			/* Insert the record into our local binary Hash table for quick	*/
			/* access.														*/
			prg_bin_hash_table_item_insert (peer_info_hash_tbl, (void *) &sta_mac_addr, peer_info_ptr, &dummy_ptr);
			}
		
		else
			{
			/* Remove the STA's record from our Hash table since it is		*/
			/* leaving.														*/
			peer_info_ptr = (WlanT_Peer_Info *) prg_bin_hash_table_item_remove (peer_info_hash_tbl, (void *) &sta_mac_addr);
			
			/* Free the memory.												*/
			op_prg_mem_free (peer_info_ptr);
			}
		}
	
	if (roam_state_ptr->scan_type == WlanC_Scan_Type_Distance && current_time >= ap_connectivity_check_time)
		{
		/* In the "virtual" scanning mode, we check if the distance of this	*/
		/* STA is greater than "start scanning" threshold. If so, we go to	*/
		/* the scan mode, and look for a new AP. 							*/
		ap_connectivity_check_time = current_time + ap_connectivity_check_interval;
		wlan_ap_eval_virtual (my_node_objid, roam_state_ptr, conn_ap_pos_info_ptr, channel_num, phy_type, rx_power_threshold);
		}

	FOUT;
	}

static void 
wlan_physical_layer_data_arrival (void)
	{
	char								msg_string [120];
	int									accept;
	OpT_Int64							dest_addr;
	OpT_Packet_Id						data_pkt_id;
	const WlanT_Control_Header_Fields*	pk_chstruct_ptr;
	WlanT_Mac_Frame_Type				rcvd_frame_type;
	WlanT_Phy_Char_Code					rcvd_frame_phy_char;
	Packet*								wlan_rcvd_frame_ptr;	
	Packet*								seg_pkptr;
	const WlanT_Beacon_Body_Fields*		pk_bbstruct_ptr;
	int									rcvd_sta_bssid;
	int									i;
	Boolean								data_pkt_received        = OPC_FALSE;
	Boolean								disable_signal_extension = OPC_FALSE;
	double								rcvd_pk_size, rx_start_time;
	double								rcvd_duration;
	char								actual_frame_name [256];

//自添加
	//$$$$$$$$$$$$$$$$$$ DSR $$$$$$$$$$$$$$$$$$$$$$$$
	//int tmp_remote_sta_addr;
	Ici* iciptr;
	Objid process_id;
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//自添加结束
	/** Process the frame received from the lower layer.          **/
	/** This routine decapsulate the frame and set appropriate    **/
	/** flags if the station needs to generate a response to the  **/
	/** received frame.											  **/
	FIN (wlan_physical_layer_data_arrival (void));

	/*  Access received packet from the physical layer stream.	*/
	wlan_rcvd_frame_ptr = op_pk_get (i_strm);	

	/* Get the type of the received WLAN frame and check 		*/
	/* whether it is marked as a bad packet in the pipeline		*/
	/* stages. Consider the possibility that we may receive a	*/
	/* noise packet from a powerful jammer, which we need		*/
	/* simply discard.											*/
	if (op_pk_encap_flag_is_set (wlan_rcvd_frame_ptr, OMSC_JAMMER_ENCAP_FLAG_INDEX) == OPC_FALSE)
		{
		op_pk_nfd_get_int32 (wlan_rcvd_frame_ptr, "Accept", &accept);
		op_pk_nfd_get_int32 (wlan_rcvd_frame_ptr, "Type", (int *) &rcvd_frame_type);
		}
	else
		{
		accept          = OPC_FALSE;
		rcvd_frame_type = WlanC_None;
		}
	
	/* If this is an ERP-OFDM packet and if we support 802.11g	*/
	/* PHY, then it is received with 6 usec signal extension	*/
	/* delay. Hence, reset the delay attribute of the packet	*/
	/* stream from the receiver.								*/
	if (accept && phy_type == WlanC_11g_PHY)
		{
		op_pk_nfd_get_int32 (wlan_rcvd_frame_ptr, "PHY Info", (int *) &rcvd_frame_phy_char);
		if (rcvd_frame_phy_char == WlanC_ERP_OFDM_11g)
			{
			op_ima_obj_attr_set (rx_state_info_ptr->mac_strm_objid, "delay", 0.0);
			
			/* If the packet will be rejected due to a 			*/
			/* collision, then update the receiver idle time	*/
			/* accordingly, since the packet could not be		*/
			/* decoded, which disables the signal extension.	*/
			if (wlan_flags->rcvd_bad_packet == OPC_TRUE)
				disable_signal_extension = OPC_TRUE;
			}
		}
	
	/* Update the receiver idle time with current time. This is	*/
	/* necessary even though the receiver may be still busy.	*/
	/* Pay attention to collided packets with signal extension.	*/
	if (disable_signal_extension == OPC_FALSE)
		rcv_idle_time = current_time;
	else if (rcv_idle_time < current_time - WLANC_11g_SIGNAL_EXTENSION)
		rcv_idle_time = current_time - WLANC_11g_SIGNAL_EXTENSION;
		
	/* If we received our own CTS-to-self and if we experienced	*/
	/* a reception during our transmission then discard the		*/
	/* CTS message since it has collided.						*/
	if (wlan_flags->rcvd_bad_cts && rcvd_frame_type == WlanC_Cts && op_pk_creation_mod_get (wlan_rcvd_frame_ptr) == my_objid)
		{
		/* Due to collision enable the EIFS duration flag.		*/ 	
		wlan_flags->wait_eifs_dur = OPC_TRUE;
			
		/* Increment the retry count.							*/
		short_retry_count++;
		
		/* Check if more retrys permitted, if not discard.		*/
		wlan_frame_discard ();

		/* Set expected frame type flag to none as we need to	*/
		/* retransmit the frame.								*/
		expected_frame_type = WlanC_None;
			
		/* Reset the NAV duration so that the retransmission is	*/
		/* not unnecessarily delayed.							*/
		if (cfp_ap_medium_control == OPC_FALSE)
			nav_duration = current_time;

		/* Reset the flag that indicated the corrupted CTS-to-	*/
		/* self message.										*/
		wlan_flags->rcvd_bad_cts = OPC_FALSE;		

		/* Printing out information to ODB.						*/
		if (wlan_trace_active == OPC_TRUE)
			{
			op_prg_odb_print_major ("Received bad CTS-to-self packet. Discarding.", OPC_NIL);
			}
		
		/* Destroy the bad packet.								*/
		op_pk_destroy (wlan_rcvd_frame_ptr);

		/* Unless it was already done, cancel the frame timeout	*/
		/* interrupt. It is likely to be still active due to	*/
		/* collision.											*/
		if (op_ev_valid (frame_timeout_evh) == OPC_TRUE)
			{
			op_ev_cancel (frame_timeout_evh);
			}
		
		/* Break the routine as no further processing is needed.*/
		FOUT;
		}
	
	/* If the packet is received while the station is in transmission,   */
	/* or if the accept field is set to false, then the packet will not  */
	/* be processed and if needed the station will retransmit the packet.*/
	if ((wlan_flags->rcvd_bad_packet == OPC_TRUE) || (accept == OPC_FALSE))
		{
		/* If the pipeline stage set the accept flag to be false or if	*/
		/* it is collided then it means that the packet is erroneous.	*/
		/* Enable the EIFS duration flag and set NAV duration to be		*/
		/* EIFS duration. Do the same if the packet has collided with	*/
		/* our transmission, which doesn't require a response, and its	*/
		/* reception ended after our transmission.						*/
		if ((accept == OPC_FALSE ||  
			 (wlan_flags->rcvd_bad_packet == OPC_TRUE && expected_frame_type == WlanC_None && 
			  op_stat_local_read (TRANSMITTER_BUSY_INSTAT) == 0)) &&
			wlan_flags->pcf_active == OPC_FALSE)
			{
			wlan_flags->wait_eifs_dur = OPC_TRUE;
     
			/* Setting nav duration to EIFS.							*/
			if (cfp_ap_medium_control == OPC_FALSE)
				{
				nav_duration = rcv_idle_time + eifs_time - difs_time;
				
				/* Set the flag that indicates an updated NAV value.	*/
				wlan_flags->nav_updated = OPC_TRUE;
				}
			}
		
		/* We may have experienced a collision during transmission. We	*/
		/* could be transmitting a packet which requires a response (an	*/
		/* Rts or a data frame requiring an Ack). Even, this is the		*/
		/* case, we do not take any action right now and wait for the	*/
		/* related timers to expire; then we will retransmit the frame.	*/
		/* This is the approach described in the standard, and it is	*/
		/* necessary because of the slight possibility that our peer	*/
		/* may receive the frame without collision and send us the		*/
		/* response back, which we should be still expecting.			*/

		/* Check whether the timer for the expected response has		*/
		/* already expired. If yes, we must initiate the retransmission.*/
		if ((expected_frame_type != WlanC_None) && (wlan_flags->transmitter_busy == OPC_FALSE) &&
			(op_ev_valid (frame_timeout_evh) == OPC_FALSE))
			{
			/* Check whether it was a PCF transmission by the AP.		*/
			if (wlan_flags->pcf_active)
				{
				/* If last frame a data frame, increment retry counter.	*/
				if ((last_frametx_type == WlanC_Data_Poll)	||	(last_frametx_type == WlanC_Data_A_P))
					{
					pcf_retry_count++;
					}

				/* Check if we were waiting for a poll response and		*/
				/* increment poll fail accordingly.						*/
				if	(((last_frametx_type == WlanC_Data_Poll)	||	(last_frametx_type == WlanC_Data_A_P)	||
					(last_frametx_type == WlanC_Cf_Poll)		||	(last_frametx_type == WlanC_Cf_A_P))	&&
					(wlan_flags->active_poll == OPC_TRUE))
					{
					/* Poll failed. Increment Failed Poll counter.		*/
					poll_fail_count++;
					wlan_flags->active_poll = OPC_FALSE;
					}
		
				/* Check if more retries are permitted, if not discard.	*/
				wlan_pcf_frame_discard ();
				}
			
			else 
				{
				/* We have either sent an RTS frame or a data frame.	*/
				/* Increment the short retry count if the transmission	*/
				/* of an RTS frame or a data frame smaller than RTS/CTS	*/
				/* threshold has failed; otherwise increment the long	*/
				/* retry count.											*/
				if (last_frametx_type == WlanC_Rts || wlan_flags->frame_size_req_rts == OPC_FALSE)
					short_retry_count++;
				else
					long_retry_count++;
				
				/* Check if more retrys permitted, if not discard */
				wlan_frame_discard ();
				}

			/* If Rts sent flag was enable then disable it as the station will recontend for the channel.	*/
			if (wlan_flags->rts_sent == OPC_TRUE)
				{
				wlan_flags->rts_sent = OPC_FALSE;
				}

			/* Set expected frame type flag to none as the		*/
			/* station needs to retransmit the frame.			*/
			expected_frame_type = WlanC_None;
			
			/* Reset the NAV duration so that the				*/
			/* retransmission is not unnecessarily delayed.		*/
			if (cfp_ap_medium_control == OPC_FALSE)
				nav_duration = current_time;
			}
		
		/* Set the bad packet dropped flag if this action is 	*/
		/* because the bad packet receive flag is true.			*/
		if (wlan_flags->rcvd_bad_packet == OPC_TRUE && op_stat_local_read (TRANSMITTER_BUSY_INSTAT) == 1)
			wlan_flags->bad_packet_dropped = OPC_TRUE;
		
		/* Reset the bad packet receive flag for subsequent		*/
		/* receptions.											*/
		wlan_flags->rcvd_bad_packet = OPC_FALSE;

		/* If the corrupted frame is a Beacon, update the		*/
		/* reliability of the signal from AP indicating that	*/
		/* the packet was not successfully received.			*/
		if (rcvd_frame_type == WlanC_Beac && roam_state_ptr->enable_roaming && roam_state_ptr->scan_type == WlanC_Scan_Type_Beacon)
			wlan_ap_reliability_eval (wlan_rcvd_frame_ptr, OPC_FALSE, roam_state_ptr);

		/* Printing out information to ODB.						*/
		if (wlan_trace_active == OPC_TRUE)
			{
			op_prg_odb_print_major ("Received bad packet. Discarding received packet", OPC_NIL);
			}
		
		/* Destroy the bad packet.								*/
		op_pk_destroy (wlan_rcvd_frame_ptr);

		/* Break the routine as no further processing is needed.*/
		FOUT;
		}

	/* If waiting for EIFS duration then set the nav duration such that	*/
	/* the normal operation is resumed.									*/
	if (wlan_flags->wait_eifs_dur == OPC_TRUE)
		{
		if (cfp_ap_medium_control == OPC_FALSE)
			nav_duration = current_time;
		
		wlan_flags->wait_eifs_dur = OPC_FALSE;
		}
	
	/* Make sure that the received packet is transmitted with a			*/
	/* physical layer technology that is supported by us so that we can	*/
	/* decode and process the packet. Get the PHY info unless it is		*/
	/* already obtained (in case of 802.11g operation).					*/
	if (phy_type != WlanC_11g_PHY)
		op_pk_nfd_get_int32 (wlan_rcvd_frame_ptr, "PHY Info", (int *) &rcvd_frame_phy_char);
	
	/* Check for the support.											*/
	if (!(rcvd_frame_phy_char == phy_char_flag || (phy_type == WlanC_11g_PHY && rcvd_frame_phy_char == WlanC_Direct_Sequence)))
		{
		/* It is an mismatching/unsupported PHY. Drop the packet and	*/
		/* quit the function.											*/
		op_pk_destroy (wlan_rcvd_frame_ptr);
		FOUT; 
		}
		
 	/* If we are in "scanning" mode, discard any packet received from	*/
	/* the physical layer, unless it is a beacon message. The packet is	*/
	/* not expected to be destined to this MAC anyway, since it is not	*/
	/* registered in a BSS that it is currently evaluating.				*/
	if (wlan_flags->scanning == OPC_TRUE && rcvd_frame_type != WlanC_Beac)
		{
		/* Printing out information to ODB.								*/
		if (wlan_trace_active == OPC_TRUE)
			{
			op_prg_odb_print_major ("Discarding the packet received from physical layer in scanning mode.", OPC_NIL);
			}
		
		/* Destroy the packet and quit the function.					*/
		op_pk_destroy (wlan_rcvd_frame_ptr);
		FOUT;
		}
	
	/* Compute the values that will be used while updating the received	*/
	/* traffic statistics.												*/
	op_pk_nfd_get_dbl (wlan_rcvd_frame_ptr, "Tx Data Rate", &rcvd_frame_drate);
	rcvd_pk_size  = (double) op_pk_total_size_get (wlan_rcvd_frame_ptr);
	rx_start_time = current_time - rcvd_pk_size / rcvd_frame_drate;
	
	/* Did we receive a data frame?										*/
	if (rcvd_frame_type & WLANC_DATA_TYPE_BIT)
		{
		WlanT_Data_Header_Fields			pk_dhstruct;
		
		/* First check that whether the station is expecting any frame or not. If not	*/
		/* then decapsulate relevant information from the packet fields and set the		*/
		/* frame response variable with appropriate frame type.							*/
		
		/* Update the data traffic received statistics since all these frame types are	*/
		/* considered as data frames by the 802.11 standard. Write the appropriate		*/
		/* values for start and end of the reception.									*/
		op_stat_write_t (data_traffic_rcvd_handle_inbits, rcvd_pk_size, rx_start_time);
		op_stat_write (data_traffic_rcvd_handle_inbits, 0.0);
		op_stat_write_t (data_traffic_rcvd_handle, 1.0, rx_start_time);
		op_stat_write (data_traffic_rcvd_handle, 0.0);
	
		/* Set the flag if the received packet is actually containing data.				*/
		if	(!(rcvd_frame_type & WLANC_NULL_DATA_BIT))
			data_pkt_received = OPC_TRUE;
		
		/* In case of Beacon colliding with other packets being received, the stations	*/
		/* will not be indicated on the PCF period. If this happens, and we receive a	*/
		/* poll packet, then we need to initialize variables that indicate the start of	*/
		/* PCF duration.							 	  								*/
		if ((cfp_ap_medium_control == OPC_FALSE) && (rcvd_frame_type & WLANC_POLL_BIT))
			{
			/* Indicate the medium controlled by the active AP.							*/
			cfp_ap_medium_control =  OPC_TRUE;
			}
	
		/* Address information, sequence control fields,	*/
		/* and the data is extracted from the rcvd packet.	*/
		{
		const WlanT_Data_Header_Fields*		pk_dhstruct_const_ptr;
		op_pk_fd_access_read_only_ptr (wlan_rcvd_frame_ptr, WLANC_DATA_HEADER_FD, (const void **) &pk_dhstruct_const_ptr);
		/* Make local copy of the sequence control fields for speed */
		pk_dhstruct = *pk_dhstruct_const_ptr;
		}

		/* Obtain the destination this packet id addressed to. */
		dest_addr = pk_dhstruct.address1;	
		remote_sta_addr = pk_dhstruct.address2;
		
		/* Check if PCF active and this is an AP. */
		if (wlan_flags->pcf_active)
			{
			/* Check if frame is from station polled and increment failed poll count if necessary	*/
			if (poll_index == -1)
				{
				/* If poll index is invalid, which should happen very rarely, that means we have	*/
				/* retransmitted a CFP frame that has failed at the end of the previous CFP.		*/
				if (remote_sta_addr == pcf_destination_addr)
					poll_fail_count = 0;
				else if (wlan_flags->active_poll == OPC_TRUE)
					poll_fail_count++;
				}
			else if (remote_sta_addr == polling_list [poll_index]) 
				{
				poll_fail_count = 0;
				}
			else if (wlan_flags->active_poll == OPC_TRUE) 
				{
				poll_fail_count++;
				}			
	
			/* Disable active poll flag since it is now satisfied one way or another */
			wlan_flags->active_poll = OPC_FALSE;
			
			if ((dest_addr != ap_mac_address)  && (dest_addr >= 0))
				{
				/* If data not addressed to AP and not broadcast then set side	*/
				/* traffic flag and set NAV to wait for expected ack 			*/
				wlan_flags->pcf_side_traf = OPC_TRUE;
				
				if (nav_duration < (current_time + sifs_time + TXTIME_CTRL (WLANC_ACK_LENGTH))) 
					nav_duration = current_time + sifs_time + TXTIME_CTRL (WLANC_ACK_LENGTH);
					
				/* Set poll more fragments flag if necessary */
				if (pk_dhstruct.more_frag == 1) 
					wlan_flags->more_frag = OPC_TRUE;
				else 
					wlan_flags->more_frag = OPC_FALSE;

				/* Set poll more data (MSDU's) flag if necessary */
				if ((pk_dhstruct.more_data == 0)  || (wlan_flags->tx_beacon == OPC_TRUE) ||
					(wlan_flags->tx_cf_end == OPC_TRUE))
					{
					/* No more polls for new MDSU will be sent to this STA so reset flag		*/
					wlan_flags->more_data = OPC_FALSE;
					}
				else 
					{
					/* need to poll this STA at least one more time, so set flag				*/
					wlan_flags->more_data = OPC_TRUE;
					}
				}
			}
		
		/* Process frame only if it is destined for this station or it	*/
		/* is a broadcast frame that is not originated from us.			*/
		if	(((dest_addr == my_address) || 
			  (dest_addr < 0 && (bss_flag == OPC_FALSE || pk_dhstruct.address3 != my_address))) && 
			 data_pkt_received)
			{	
			/* Make sure that we didn't receive a QoS Data frame.		*/
			if (rcvd_frame_type == WlanC_QoS_Data) 
				wlan_error_print ("QoS data frame is received by nQSTA!!!", "", "");
	
			/* Extracting the MSDU from the packet only if the packet	*/
			/* is destined for this station.							*/		
			op_pk_fd_get_pkt (wlan_rcvd_frame_ptr, WLANC_DATA_BODY_FD, &seg_pkptr);

			/* Only send acknowledgement if the data frame is destined for 	*/
			/* this station. No Acks for broadcast frame.					*/
			if (dest_addr == my_address)
				{
				/* Send the acknowledgement to any received data frame.	*/
				fresp_to_send = WlanC_Ack;
				
				/* If no more fragments, then duration value in ack must be zero */
				if (pk_dhstruct.more_frag == 0)
					wlan_flags->duration_zero = OPC_TRUE;
				else 
					wlan_flags->duration_zero = OPC_FALSE;							
				}
			             	
			/* If PCF is active and this is an AP */ 
			if (wlan_flags->pcf_active)
				{
				/* Set poll more fragments flag if necessary */
				if (pk_dhstruct.more_frag == 1)
					wlan_flags->more_frag = OPC_TRUE;
				else 
					wlan_flags->more_frag = OPC_FALSE;

				/* Set poll more data (MSDU's) flag if necessary */
				if ((pk_dhstruct.more_data == 0)  || (wlan_flags->tx_beacon == OPC_TRUE) ||
					(wlan_flags->tx_cf_end == OPC_TRUE))
					{
					/* no more polls for new MDSU will be sent to this STA so reset flag		*/
					wlan_flags->more_data = OPC_FALSE;
					}
				else 
					{
					/* need to poll this STA at least one more time, so set flag				*/
					wlan_flags->more_data = OPC_TRUE;
					}
				}
												
			/* If its a duplicate packet then destroy it and do nothing, otherwise 	*/
			/* insert it in the defragmentation list (reassembly buffer).			*/
			if (wlan_tuple_find (remote_sta_addr, pk_dhstruct.sequence_control, dest_addr, pk_dhstruct.retry) == OPC_FALSE)
				{
	           	/* Check whether congestion area analysis is enabled. 				*/ 
				if (CONGESTION_AREAS_ENABLED)
					{
					/* If this is a tracer packet update the congestion area info.	*/
					if (op_pk_encap_flag_is_set (seg_pkptr, OMSC_BGUTIL_ENCAP_FLAG_INDEX))
						wlan_support_congestion_area_info_update (seg_pkptr, rx_state_info_ptr);
					}
				
				/* Data packet id of the received data frame is extracted.			*/
				op_pk_fd_get_pkid (wlan_rcvd_frame_ptr, WLANC_DATA_PKID_FD, &data_pkt_id);

				/* Process the received MPDU. If contains an entire MSDU or			*/
				/* completes the previously received fragments of an MSDU, then		*/
				/* forward the MSDU to the higher layer or to its final destination */
				/* (if we are an AP and the MSDU is destined for an STA in our BSS).*/
				wlan_data_process (seg_pkptr, dest_addr, remote_sta_addr, pk_dhstruct.address3, pk_dhstruct.hl_protocol,
					pk_dhstruct.sequence_control & WLANC_FRAG_NUM_BIT_MASK, pk_dhstruct.more_frag, data_pkt_id);
				}
			else
				{
				/* Destroy the duplicate packet without any processing.				*/
				op_pk_destroy (seg_pkptr);
				}
			}
			
		/* Check whether we are being polled.										*/
		if ((rcvd_frame_type & WLANC_POLL_BIT) && dest_addr == my_address)
			{
			/* Since frame contained a poll for my address, set polled flag.		*/
			wlan_flags->polled = OPC_TRUE;
			
			/* Printing out information to ODB.	*/
			if (wlan_trace_active == OPC_TRUE)
				{
				wlan_frame_type_conv(rcvd_frame_type, actual_frame_name);
				
				sprintf (msg_string, "%s is received from AP", actual_frame_name);
				op_prg_odb_print_major (msg_string, OPC_NIL);
			 	}
			}
		
		if (expected_frame_type == WlanC_Ack)
			{
			/* If an Ack is expected, and a PCF Ack is received, regardless of whom	*/  
			/* the frame is	addressed to, process the received ACK.					*/
			if (rcvd_frame_type & WLANC_ACK_BIT)  
				{
				/* Printing out information to ODB.	*/
				if (wlan_trace_active == OPC_TRUE)
					{
					if ((last_frametx_type == WlanC_Data) || (last_frametx_type == WlanC_Data_Ack))
						{
						sprintf (msg_string, "Ack received for data packet " OPC_PACKET_ID_FMT, pkt_in_service);
						op_prg_odb_print_major (msg_string, OPC_NIL);
						}
					else
						{
						sprintf (msg_string, "Ack received for data packet " OPC_PACKET_ID_FMT, pcf_pkt_in_service);
						op_prg_odb_print_major (msg_string, OPC_NIL);
						}
					}			

				op_stat_write (retrans_handle, (double) (pcf_retry_count));
				op_stat_write (global_retrans_handle, (double) (pcf_retry_count));				

				if ((last_frametx_type == WlanC_Data) || (last_frametx_type == WlanC_Data_Ack))
					{
					/* Reset the retry counts as the expected frame is received.		*/
					short_retry_count = 0;
					long_retry_count  = 0;
					
					/* Similarly reset the variables that may have been set and used	*/
					/* for this transmission during the DCF period that proceeded the	*/
					/* current PCF period.												*/
					wlan_flags->rts_sent = OPC_FALSE;
					if (wlan_flags->cw_required != OPC_TRUE)
						backoff_slots = BACKOFF_SLOTS_UNSET;
					
					/* Decrement number of fragment count because one fragment is		*/
					/* successfully transmitted.										*/
					num_fragments = num_fragments - 1;				

					/* If the transmission of the MSDU completed, update the higher		*/
					/* layer data queue statistic and queue size information.			*/
					if (num_fragments == 0)
						{
						total_hlpk_num--;
						total_hlpk_size -= packet_size_dcf;
						op_stat_write (hl_packets_rcvd, (double) total_hlpk_num);
						}
					
					/* Data packet is successfully delivered to remote station,			*/
					/* since no further retransmission is needed the copy of the data	*/
					/* packet will be destroyed.										*/
					if (wlan_transmit_frame_copy_ptr != OPC_NIL) 
						{
						op_pk_destroy (wlan_transmit_frame_copy_ptr);
						wlan_transmit_frame_copy_ptr = OPC_NIL;
//自添加
						//$$$$$$$$$$$$$$$$$$ DSR $$$$$$$$$$$$$$$$$$$$$$$$
						// DSR: send an ack intrpt and an ici to the DSR routing layer
						if ((num_fragments==0)&&(data_packet_type==DATA_PACKET_TYPE))
						{
							printf("--- MAC Msg @ node %d --- sending ACK for destination %d\n", my_address,data_packet_dest);
							iciptr = op_ici_create("aodv_ack");
							op_ici_attr_set(iciptr,"Relay_Destination",data_packet_dest);    
							op_ici_attr_set(iciptr,"Final_Destination",data_packet_final_dest);
							op_ici_install(iciptr);
							// schedule an intrpt for the DSR process model	
							process_id = op_id_from_name(op_topo_parent (op_id_self()),OPC_OBJTYPE_QUEUE,"aodv_routing");
							op_intrpt_schedule_remote(op_sim_time(),ACK_CODE,process_id);
							if (1)
							{
								op_prg_odb_bkpt("ack_mac");
							}
						}
						//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//自添加结束
						}
					}
				else
					{
					/* Reset the retry counter as the expected frame is received.		*/
					pcf_retry_count = 0;

					/* Decrement number of fragment count because one fragment is		*/
					/* successfully transmitted.							      		*/
					pcf_num_fragments = pcf_num_fragments - 1;				

					/* If the transmission of the MSDU completed, update the higher		*/
					/* layer data queue statistic and queue size information.			*/
					if (pcf_num_fragments == 0)
						{
						total_hlpk_num--;
						total_hlpk_size -= packet_size_pcf;
						op_stat_write (hl_packets_rcvd, (double) total_hlpk_num);
						}	

					/* Data packet is successfully delivered to remote station,			*/
					/* since no further retransmission is needed the copy of the data	*/
					/* packet will be destroyed.										*/
					if (wlan_pcf_transmit_frame_copy_ptr != OPC_NIL)
						{
						op_pk_destroy (wlan_pcf_transmit_frame_copy_ptr);
						wlan_pcf_transmit_frame_copy_ptr = OPC_NIL;

						}
					}
				}
			else
				{
				/* A frame has not been properly acknowledged and requires retransmission	*/
				/* If the last transmitted frame was a data frame must be a DCF				*/
				/* transmission. Else must be a PCF transmission.							*/
				if ((last_frametx_type == WlanC_Data) || (last_frametx_type == WlanC_Data_Ack)) 
					{
					/* Did the data transmission fail in spite of a successful RTS/CTS		*/
					/* exchange?															*/
					if (wlan_flags->frame_size_req_rts)
						/* Increment the long retry count.									*/
						long_retry_count++;
					else
						/* Increment the short retry count.									*/
						short_retry_count++;
					
					/* Reset the rts_sent flag since we will recontend for the medium.		*/
					wlan_flags->rts_sent = OPC_FALSE;					
					}
				else 
					{
					pcf_retry_count++;
					}
				}
			}

		if 	((expected_frame_type == WlanC_Data) && 
			((rcvd_frame_type == WlanC_Data) ||	(rcvd_frame_type == WlanC_Data_Null)))
			{
			/* If PCF Data is expected, and some type of PCF Data is received regardless of		*/  
			/* whom addressed to, poll is satisfied so destroy copy of tx packet .				*/
			if (((last_frametx_type == WlanC_Data)  || 
				(last_frametx_type == WlanC_Data_Ack)) && 
				(wlan_transmit_frame_copy_ptr != OPC_NIL)) 
				{
				op_pk_destroy (wlan_transmit_frame_copy_ptr);
				wlan_transmit_frame_copy_ptr = OPC_NIL;
				}
			else if (wlan_pcf_transmit_frame_copy_ptr != OPC_NIL)
				{
				op_pk_destroy (wlan_pcf_transmit_frame_copy_ptr);
				wlan_pcf_transmit_frame_copy_ptr = OPC_NIL;
				}
			}
		
		if (expected_frame_type == WlanC_Cts) 
			{
			/* Since the station did not receive the expected frame	*/
			/* it has to retransmit the packet.						*/
			short_retry_count++;
						
			if (cfp_ap_medium_control == OPC_FALSE)
				nav_duration = current_time;
			}
			
		/* Update NAV duration if the received NAV duration is greater	*/
		/* than the current NAV duration.							  	*/    	
		if ((pk_dhstruct.duration < 32768) && 
			(nav_duration < (pk_dhstruct.duration + current_time)))
			{
			nav_duration = pk_dhstruct.duration + current_time;

			/* Set the flag that indicates updated NAV value.			*/
			wlan_flags->nav_updated = OPC_TRUE;
			}
		else if (nav_duration < current_time)
			{
			/* Update NAV to current time in case ack is required (otherwise negative duration may be	*/
			/* be computed. Should not effect deference processing, so don't set nav_updated flag.		*/
			nav_duration = current_time;
			}			
		}

	else if	(rcvd_frame_type == WlanC_Beac)
		{
		WlanT_Data_Header_Fields			pk_dhstruct;
		
		/* We received a beacon frame. Update received management	*/
		/* traffic statistics.										*/
		op_stat_write_t (mgmt_traffic_rcvd_handle_inbits, rcvd_pk_size, rx_start_time);
		op_stat_write (mgmt_traffic_rcvd_handle_inbits, 0.0);
		op_stat_write_t (mgmt_traffic_rcvd_handle, 1.0, rx_start_time);
		op_stat_write (mgmt_traffic_rcvd_handle, 0.0);
			
		/* Address information, sequence control fields nd the data	*/
		/* is extracted from the rcvd packet.						*/
		{
		const WlanT_Data_Header_Fields*		pk_dhstruct_const_ptr;
		op_pk_fd_access_read_only_ptr (wlan_rcvd_frame_ptr, WLANC_DATA_HEADER_FD, (const void **) &pk_dhstruct_const_ptr);
		/* Make local copy of the sequence control fields for speed */
		pk_dhstruct = *pk_dhstruct_const_ptr;
		}

		/* The destination address in this case will be -1 as the 	*/
		/* beacon frame is a broadcast frame.				      	*/
		dest_addr = pk_dhstruct.address1;			
		
		/* Store the address of the AP transmitting the Beacon.		*/
		remote_sta_addr = pk_dhstruct.address2;	
		rcvd_sta_bssid = pk_dhstruct.address3;		

		/* If roaming enabled, update the reliability of the signal	*/
		/* from the AP that sent this Beacon, which we received		*/
		/* successfully.											*/
		if (roam_state_ptr->enable_roaming && roam_state_ptr->scan_type == WlanC_Scan_Type_Beacon)
			wlan_ap_reliability_eval (wlan_rcvd_frame_ptr, OPC_TRUE, roam_state_ptr);
		
		/* If the STA is scanning for a new BSS, then save the BSS ID of the beacon transmitter. */
		if (roam_state_ptr->scan_mode)
			eval_bss_id = rcvd_sta_bssid;

		/* Since beacons are broadcasted, no response is needed.	*/
		fresp_to_send = WlanC_None; 
		
		/* Extracting the Beacon Body packet.						*/
		op_pk_fd_get_pkt (wlan_rcvd_frame_ptr, WLANC_DATA_BODY_FD, &seg_pkptr);

		/* Extracting the Beacon Body structure from packet.		*/
		op_pk_fd_access_read_only_ptr (seg_pkptr, WLANC_BEACON_BODY_FD, (const void **) &pk_bbstruct_ptr);
			
		if (expected_frame_type != WlanC_None) 
			{
			/* Since the station did not receive the expected frame	*/
			/* it has to retransmit the packet.						*/
			if (wlan_flags->pcf_active == OPC_TRUE) 
				{
				pcf_retry_count++;
				}
			else
				{
				/* Increment the short retry count if the transmission	*/
				/* of an RTS frame or a data frame smaller than RTS/CTS	*/
				/* threshold has failed; otherwise increment the long	*/
				/* retry count.											*/
				if (last_frametx_type == WlanC_Rts || wlan_flags->frame_size_req_rts == OPC_FALSE)
					short_retry_count++;
				else
					long_retry_count++;
					
				/* Also reset the rts_sent flag since we will recontend.*/
				wlan_flags->rts_sent = OPC_FALSE;
				}
			}
			
		/* If beacon is intended for this BSS, update BSS related variables.	*/
		if (rcvd_sta_bssid == bss_id) 
			{
			/* Get the beacon interval information from the beacon.				*/
			beacon_int = pk_bbstruct_ptr->beacon_intv;				
	
			/* Check whether this beacons indicates start of a CFP.				*/
			/* Indicate the presence of an active AP  */
			if (pk_bbstruct_ptr->cf_par.cfp_count == 0)
				cfp_ap_medium_control = OPC_TRUE;
			
			/* If we and our AP support 11g data rates and PHY then compare the	*/
			/* value of non_erp_present in the beacon frame with our own flag.	*/
			if (phy_type == WlanC_11g_PHY && ap_peer_info_ptr->is_erp == OPC_TRUE && roam_state_ptr->scan_mode == OPC_FALSE)
				{
				if (!wlan_flags->non_erp_present && pk_bbstruct_ptr->non_erp_present)
					{
					/* There are new non-ERP STAs in our BSS as reported by our	*/
					/* AP. Increase the slot time and recompute the dependent	*/
					/* parameters.												*/
					wlan_slot_time_set (20E-06);

					/* Reduce the control frame data rate to 802.11/11b			*/
					/* mandatory data rate.										*/
					control_data_rate = WLANC_11b_MIN_MANDATORY_DRATE;
					
					/* Set our own flag.										*/
					wlan_flags->non_erp_present = OPC_TRUE;
					}
				
				else if (wlan_flags->non_erp_present && !pk_bbstruct_ptr->non_erp_present)
					{
					/* There are no non-ERP STAs left in our BSS as reported by	*/
					/* our AP. Decrease the slot time and recompute the			*/
					/* dependent parameters.									*/
					wlan_slot_time_set (9E-06);

					/* Reselect the control frame data rate. Choose the highest	*/
					/* mandatory data rate that is equal to or lower than the	*/
					/* data rate specified for data transmissions.				*/
					for (i = 0; data_tx_rate < WLANC_11g_MANDATORY_DRATE_ARRAY [i]; i++);
					control_data_rate = WLANC_11g_MANDATORY_DRATE_ARRAY [i];
					
					/* Reset our own flag.										*/
					wlan_flags->non_erp_present = OPC_FALSE;
					}
				}
			}

		if ((pk_bbstruct_ptr->cf_par.cfp_count == 0) && 
			((pk_bbstruct_ptr->timestamp + pk_bbstruct_ptr->cf_par.cfp_durremaining) > nav_duration))
			{
			nav_duration = pk_bbstruct_ptr->timestamp + pk_bbstruct_ptr->cf_par.cfp_durremaining;

			/* Set the flag that indicates updated NAV value.			*/
			wlan_flags->nav_updated = OPC_TRUE;
			
			}

		/* Update nav duration if the received nav duration is greater	*/
		/* than the current nav duration.							  	*/    	
		else if ((pk_dhstruct.duration < 32768) && 
			(nav_duration < (pk_dhstruct.duration + current_time)))
			{
			nav_duration = pk_dhstruct.duration + current_time;

			/* Set the flag that indicates updated NAV value.			*/
			wlan_flags->nav_updated = OPC_TRUE;			
			}

		/* Destroy beacon body since no longer needed */
		op_pk_destroy (seg_pkptr);
		
		/* Printing out information to ODB.	*/
		if (wlan_trace_active == OPC_TRUE)
			{
			op_prg_odb_print_major ("Beacon frame is received", OPC_NIL);
		 	}
		}

	else if	(rcvd_frame_type == WlanC_Rts)
		{
		/* Update received control traffic statistics. Write the		*/
		/* appropriate values for start and end of the reception.		*/
		op_stat_write_t (ctrl_traffic_rcvd_handle_inbits, rcvd_pk_size, rx_start_time);
		op_stat_write (ctrl_traffic_rcvd_handle_inbits, 0.0);
		op_stat_write_t (ctrl_traffic_rcvd_handle, 1.0, rx_start_time);
		op_stat_write (ctrl_traffic_rcvd_handle, 0.0);

		/* First check that whether the station is expecting any frame or not	*/
		/* If not then decapsulate the Rts frame and set a Cts frame response	*/
		/* if frame is destined for this station. Otherwise, just update the	*/
		/* network allocation vector for this station.							*/
		op_pk_fd_access_read_only_ptr (wlan_rcvd_frame_ptr, WLANC_CNTL_HEADER_FD, (const void **) &pk_chstruct_ptr);			
		dest_addr = pk_chstruct_ptr->rx_addr;
		remote_sta_addr = pk_chstruct_ptr->tx_addr;
			
		if (expected_frame_type == WlanC_None)
			{
			/* We will respond to the Rts with a Cts only if a) the	*/
			/* Rts is destined for us, and b) our NAV duration is	*/
			/* not larger than current simulation time and c) our	*/
			/* receiver is not busy									*/
			if ((my_address == dest_addr) && (current_time >= nav_duration))
				{
				/* Only if the receiver is idle do we response to the 	*/
				/* RTS frame.											*/
				if (wlan_flags->receiver_busy == OPC_FALSE)
					{
					/* Set the frame response field to Cts.				*/
					fresp_to_send = WlanC_Cts;

					/* Printing out information to ODB.					*/
					if (wlan_trace_active == OPC_TRUE)
						op_prg_odb_print_major ("RTS is received and CTS will be transmitted.", OPC_NIL);
					}
				
				/* Printing out information to ODB.						*/
				else if (wlan_trace_active == OPC_TRUE)
					{
					op_prg_odb_print_major ("RTS is received and ignored due to busy receiver.", OPC_NIL);
					}
				}			
			else
				{
				/* Printing out information to ODB.					*/
				if (wlan_trace_active == OPC_TRUE)
					op_prg_odb_print_major ("RTS is received and discarded.", OPC_NIL);
				}				
		 	}
		else
			{				
			/* Since the station did not receive the expected frame it	*/
			/* has to retransmit the packet. Increment the appropriate	*/
			/* retry count.												*/
			if (wlan_flags->pcf_active == OPC_TRUE) 
				{
				pcf_retry_count++;
				}
			else
				{
				/* Increment the short retry count if the transmission	*/
				/* of an RTS frame or a data frame smaller than RTS/CTS	*/
				/* threshold has failed; otherwise increment the long	*/
				/* retry count.											*/
				if (last_frametx_type == WlanC_Rts || wlan_flags->frame_size_req_rts == OPC_FALSE)
					short_retry_count++;
				else
					long_retry_count++;
					
				/* Also reset the rts_sent flag since we will recontend.*/
				wlan_flags->rts_sent = OPC_FALSE;
				}
				
			/* Reset the NAV duration so that the				*/
			/* retransmission is not unnecessarily delayed.		*/
			if (cfp_ap_medium_control == OPC_FALSE)
				nav_duration = current_time;
						
			/* Reset the expected frame type variable since we	*/
			/* will retransmit.									*/
			fresp_to_send = WlanC_None;
			}

		/* Update nav_duration if the received duration is greater than	*/
		/* the current nav_duration, unless the RTS is destined to us	*/
		/* and we are not responding with a CTS.						*/    	
		if (nav_duration < (pk_chstruct_ptr->duration + current_time) && (dest_addr != my_address || fresp_to_send == WlanC_Cts))
			{
			double tmp_drate;
			
			nav_duration = pk_chstruct_ptr->duration + current_time;

			/* Set the flag that indicates updated NAV value.			*/
			wlan_flags->nav_updated = OPC_TRUE;
			
			/* Since we are updating our NAV based on an RTS we			*/
			/* received, we can reset it if we don't detect any			*/
			/* activity on the medium for a "2 SIFS + 2 SLOT + CTS_TX"	*/
			/* time, as described in section 9.2.5.4. Hence schedule	*/
			/* a self interrupt for NAV reset time.						*/
			tmp_drate = rcvd_frame_drate;
			nav_reset_evh = op_intrpt_schedule_self (current_time + 2 * sifs_time + TXTIME_CTRL_DR (WLANC_CTS_LENGTH, tmp_drate) + 2 * slot_time,
													 WlanC_NAV_Reset_Time);
			}				
		}		
	
	else if	(rcvd_frame_type == WlanC_Cts)
		{
		/* Update received control traffic statistics. Write the		*/
		/* appropriate values for start and end of the reception.		*/
		op_stat_write_t (ctrl_traffic_rcvd_handle_inbits, rcvd_pk_size, rx_start_time);
		op_stat_write (ctrl_traffic_rcvd_handle_inbits, 0.0);
		op_stat_write_t (ctrl_traffic_rcvd_handle, 1.0, rx_start_time);
		op_stat_write (ctrl_traffic_rcvd_handle, 0.0);

		/* First check that whether the station is expecting any frame or not	*/
		/* If not then decapsulate the RTS frame and set a CTS frame response	*/
		/* if frame is destined for this station. Otherwise, just update the	*/
		/* network allocation vector for this station.							*/
		op_pk_fd_access_read_only_ptr (wlan_rcvd_frame_ptr, WLANC_CNTL_HEADER_FD, (const void **) &pk_chstruct_ptr);
		dest_addr = pk_chstruct_ptr->rx_addr;

		/* If the frame is destined for this station and the station is expecting	*/
		/* CTS frame and the receiver is free, then set appropriate indicators.		*/
		if ((dest_addr == my_address) && (expected_frame_type == rcvd_frame_type) &&
			(wlan_flags->receiver_busy == OPC_FALSE)) 				
			{
			/* The receipt of CTS frame indicates that RTS is successfully	*/
			/* transmitted and the station can now respond with Data frame	*/
			fresp_to_send = WlanC_Data;

			/* Set the flag indicating that Rts is successfully transmitted.*/
			wlan_flags->rts_sent = OPC_TRUE;

			/* Printing out information to ODB.								*/
			if (wlan_trace_active == OPC_TRUE)
				{	
				sprintf (msg_string, "CTS is received for Data packet " OPC_PACKET_ID_FMT, pkt_in_service);
				op_prg_odb_print_major (msg_string, OPC_NIL);
				}
			}
		else
			{
			/* Printing out information to ODB.								*/
			if (wlan_trace_active == OPC_TRUE)
				op_prg_odb_print_major ("Cts is received and discarded.", OPC_NIL);

			/* Check whether we were expecting another frame. If yes then	*/
			/* we need to retransmit the frame for which we were expecting	*/
			/* a reply.														*/
			if (expected_frame_type != WlanC_None)
				{				
				/* Since the station did not receive the expected frame it	*/
				/* has to retransmit the packet. Increment the appropriate	*/
				/* retry count.												*/
				if (wlan_flags->pcf_active == OPC_TRUE) 
					{
					pcf_retry_count++;
					}
				else
					{
					/* Increment the short retry count if the transmission	*/
					/* of an RTS frame or a data frame smaller than RTS/CTS	*/
					/* threshold has failed; otherwise increment the long	*/
					/* retry count.											*/
					if (last_frametx_type == WlanC_Rts || wlan_flags->frame_size_req_rts == OPC_FALSE)
						short_retry_count++;
					else
						long_retry_count++;
					
					/* Also reset the rts_sent flag since we will recontend.*/
					wlan_flags->rts_sent = OPC_FALSE;
					}
				
				/* Reset the NAV duration so that the retransmission is not	*/
				/* unnecessarily delayed.									*/
				if (cfp_ap_medium_control == OPC_FALSE)
					nav_duration = current_time;
				}
			}
						
		/* If network allocation vector is less than the received duration	*/
		/* value then update its value.  									*/
		if (nav_duration < (pk_chstruct_ptr->duration + current_time))
			{
			nav_duration = pk_chstruct_ptr->duration + current_time;				
			
			/* Set the flag that indicates updated NAV value.				*/
			wlan_flags->nav_updated = OPC_TRUE;
			}			
		}
	
	else if	((rcvd_frame_type == WlanC_Ack)	|| (rcvd_frame_type == WlanC_Cf_End) ||
			(rcvd_frame_type == WlanC_Cf_End_A))
		{				
		op_pk_fd_access_read_only_ptr (wlan_rcvd_frame_ptr, WLANC_CNTL_HEADER_FD, (const void **) &pk_chstruct_ptr); 		
		dest_addr = pk_chstruct_ptr->rx_addr;
		
		/* If PCF active and this is an AP */
		if (wlan_flags->pcf_active)
			{
			/* Check if frame is intended for this station and increment failed poll count if necessary */
			if (dest_addr == my_address) 
				poll_fail_count = 0;
			else if (wlan_flags->active_poll == OPC_TRUE) 
				{
				poll_fail_count++;
				}

			wlan_flags->active_poll = OPC_FALSE;
			
			/* Clear side traffic flag if set since Ack is now received.*/
			wlan_flags->pcf_side_traf = OPC_FALSE;
			}
			
		/* Update received control traffic statistics. Write the		*/
		/* appropriate values for start and end of the reception.		*/
		op_stat_write_t (ctrl_traffic_rcvd_handle_inbits, rcvd_pk_size, rx_start_time);
		op_stat_write (ctrl_traffic_rcvd_handle_inbits, 0.0);
		op_stat_write_t (ctrl_traffic_rcvd_handle, 1.0, rx_start_time);
		op_stat_write (ctrl_traffic_rcvd_handle, 0.0);

		if (expected_frame_type == WlanC_Ack)
			{
			/* If an Ack is expected, and a DCF Ack is received for this destination, or 	*/  
			/* CF_END+ACK is received, process the received ack.                            */
			if (((rcvd_frame_type == WlanC_Ack) && (dest_addr == my_address)) || 
				(rcvd_frame_type == WlanC_Cf_End_A))
				{
				/* Printing out information to ODB.	*/
				if (wlan_trace_active == OPC_TRUE)
					{
					if ((last_frametx_type == WlanC_Data) || (last_frametx_type == WlanC_Data_Ack))
						{
						sprintf (msg_string, "Ack received for data packet " OPC_PACKET_ID_FMT, pkt_in_service);
						op_prg_odb_print_major (msg_string, OPC_NIL);
						}
					else
						{
						sprintf (msg_string, "Ack received for data packet " OPC_PACKET_ID_FMT, pcf_pkt_in_service);
					  	op_prg_odb_print_major (msg_string, OPC_NIL);
						}
					}			

				if ((last_frametx_type == WlanC_Data) || (last_frametx_type == WlanC_Data_Ack))
					{
					/* Write the number of retransmission attempts into its			*/
					/* statistic.													*/
					op_stat_write (retrans_handle, 			(double) (short_retry_count + long_retry_count));
					op_stat_write (global_retrans_handle, 	(double) (short_retry_count + long_retry_count));					
					
					/* Reset the retry counts as the expected frame is received.	*/
					short_retry_count = 0;
					long_retry_count  = 0;
					
					/* Decrement number of fragment count because one fragment is successfully transmitted.	*/
					num_fragments = num_fragments - 1;				
					}
				else
		        	{
					op_stat_write (retrans_handle, 			(double) (pcf_retry_count));
					op_stat_write (global_retrans_handle, 	(double) (pcf_retry_count));					

					/* Reset the retry counter as the expected frame is received	*/
					pcf_retry_count = 0;

					/* Decrement number of fragment count because one fragment is successfully transmitted.	*/
					pcf_num_fragments = pcf_num_fragments - 1;
					
					/* If the transmission of the MSDU completed, update the higher		*/
					/* layer data queue statistic and queue size information.			*/
					if (pcf_num_fragments == 0)
						{
						total_hlpk_num--;
						total_hlpk_size -= packet_size_pcf;
						op_stat_write (hl_packets_rcvd, (double) total_hlpk_num);
						}	
                    }

				/* When there are no more fragments to transmit then disable the RTS sent flag	*/
				/* if it was enabled because the contention period due to RTS/CTS exchange is 	*/
				/* over and another RTS/CTS exchange is needed for next contention period.		*/
				if (num_fragments == 0)
					{
					wlan_flags->rts_sent = OPC_FALSE;
					
					/* Set the contention window flag. Since the ACK for the last 		*/
					/* fragment indicates a	successful transmission of the entire data,	*/
					/* we need to back-off for a contention window period.				*/
					if (rcvd_frame_type == WlanC_Ack)
						wlan_flags->cw_required = OPC_TRUE;					

					/* Since the transmission of the higher layer packet is	*/
					/* complete, update the queue size information and		*/
					/* statistic.											*/
					total_hlpk_num--;
					total_hlpk_size -= packet_size_dcf;
					op_stat_write (hl_packets_rcvd, (double) total_hlpk_num);
					}

				/* Data packet is successfully delivered to remote station,			*/
				/* since no further retransmission is needed the copy of the data	*/
				/* packet will be destroyed.										*/
				if (((last_frametx_type == WlanC_Data) || (last_frametx_type == WlanC_Data_Ack)) &&
					(wlan_transmit_frame_copy_ptr != OPC_NIL)) 
					{
					op_pk_destroy (wlan_transmit_frame_copy_ptr);
					wlan_transmit_frame_copy_ptr = OPC_NIL;
//自添加
						//$$$$$$$$$$$$$$$$$$ DSR $$$$$$$$$$$$$$$$$$$$$$$$
						// DSR: send an ack intrpt and an ici to the DSR routing layer
						if ((num_fragments==0)&&(data_packet_type==DATA_PACKET_TYPE))
						{
							printf("--- MAC Msg @ node %d --- sending ACK for destination %d\n", my_address,data_packet_dest);
							iciptr = op_ici_create("aodv_ack");
							op_ici_attr_set(iciptr,"Relay_Destination",data_packet_dest);    
							op_ici_attr_set(iciptr,"Final_Destination",data_packet_final_dest);
							op_ici_install(iciptr);
							// schedule an intrpt for the DSR process model	
							process_id = op_id_from_name(op_topo_parent (op_id_self()),OPC_OBJTYPE_QUEUE,"aodv_routing");
							op_intrpt_schedule_remote(op_sim_time(),ACK_CODE,process_id);
							if (1)
							{
								op_prg_odb_bkpt("ack_mac");
							}
						}
						//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//自添加结束
					}
				else if (wlan_pcf_transmit_frame_copy_ptr != OPC_NIL)
					{
					op_pk_destroy (wlan_pcf_transmit_frame_copy_ptr);
					wlan_pcf_transmit_frame_copy_ptr = OPC_NIL;
					}
				}
			else
				{
				/* We didn't receive the ACK we were expecting.			*/
				/* Increment the long retry count if the transmission	*/
				/* of a data frame larger than RTS/CTS threshold has	*/
				/* failed; otherwise increment the short retry count.	*/
				if (wlan_flags->frame_size_req_rts)
					long_retry_count++;
				else 
					short_retry_count++;
				
				/* Also reset the rts_sent flag since we will recontend.*/
				wlan_flags->rts_sent = OPC_FALSE;
				
				/* Printing out information to ODB.						*/
				if (wlan_trace_active == OPC_TRUE)
					{
					op_prg_odb_print_major ("The expected ACK is not received.", OPC_NIL);
					}
				}
			}

		if (expected_frame_type == WlanC_Cts) 
			{
			/* Since the station did not receive the expected frame	*/
			/* it has to retransmit the packet.						*/
			short_retry_count++;
			
			/* Reset the NAV duration so that the				*/
			/* retransmission is not unnecessarily delayed.		*/
			if (cfp_ap_medium_control == OPC_FALSE)
				nav_duration = current_time;
			}
		
		if ((rcvd_frame_type == WlanC_Cf_End) || (rcvd_frame_type == WlanC_Cf_End_A))
			{
			/* If a CFP end frame was received, indicate that availability	*/
			/* of the medium to the stations to contend to transmit frames.	*/
			cfp_ap_medium_control = OPC_FALSE;
			
			/* A CF-End(+)ACK message received from any BSS resets the NAV	*/
			/* (section 9.3.2.2).											*/
			nav_duration = current_time;
		
			/* Set the flag that indicates updated NAV value.				*/
			wlan_flags->nav_updated = OPC_TRUE;

			/* Printing out information to ODB.								*/
			if (wlan_trace_active == OPC_TRUE)
				{
				if (rcvd_frame_type == WlanC_Cf_End)
					op_prg_odb_print_major ("CF-End frame is received.",     OPC_NIL);
				else
					op_prg_odb_print_major ("CF-End+Ack frame is received.", OPC_NIL);
				}				
			}	

		/* If network allocation vector is less than the received duration	*/
		/* value then update its value. 									*/
		if (rcvd_frame_type == WlanC_Ack && nav_duration < (pk_chstruct_ptr->duration + current_time))
			{
			nav_duration = pk_chstruct_ptr->duration + current_time;

			/* Set the flag that indicates updated NAV value.				*/
			wlan_flags->nav_updated = OPC_TRUE;
			}		
		}

	
	/* Check whether we received an 802.11e frame, which we don't support.	*/
	else if (rcvd_frame_type == WlanC_BAR || rcvd_frame_type == WlanC_BA || rcvd_frame_type == WlanC_Action)
		{
		/* Update the related statistics.									*/
		if (rcvd_frame_type == WlanC_Action)
			{
			op_stat_write_t (mgmt_traffic_rcvd_handle_inbits, rcvd_pk_size, rx_start_time);
			op_stat_write (mgmt_traffic_rcvd_handle_inbits, 0.0);
			op_stat_write_t (mgmt_traffic_rcvd_handle, 1.0, rx_start_time);
			op_stat_write (mgmt_traffic_rcvd_handle, 0.0);
			}
		else
			{
			op_stat_write_t (ctrl_traffic_rcvd_handle_inbits, rcvd_pk_size, rx_start_time);
			op_stat_write (ctrl_traffic_rcvd_handle_inbits, 0.0);
			op_stat_write_t (ctrl_traffic_rcvd_handle, 1.0, rx_start_time);
			op_stat_write (ctrl_traffic_rcvd_handle, 0.0);
			}
		
		/* Get the duration and destination address information.			*/
		if (rcvd_frame_type == WlanC_Action)
			{
			const WlanT_Data_Header_Fields*		pk_dhstruct_ptr;
			op_pk_fd_access_read_only_ptr (wlan_rcvd_frame_ptr, WLANC_DATA_HEADER_FD, (const void **) &pk_dhstruct_ptr);
			dest_addr     = pk_dhstruct_ptr->address1;
			rcvd_duration = pk_dhstruct_ptr->duration;
			}
		else
			{
			op_pk_fd_access_read_only_ptr (wlan_rcvd_frame_ptr, WLANC_CNTL_HEADER_FD, (const void **) &pk_chstruct_ptr);
			dest_addr     = pk_chstruct_ptr->rx_addr;
			rcvd_duration = pk_chstruct_ptr->duration;
			}
		
		/* An 11e specific frame can't be addresses to us.					*/
		if (dest_addr == my_address)
			wlan_error_print ("An 802.11e specific frame is addresses to an nQSTA.", "", "");
		
		/* Check whether we were expecting another frame. If yes then we	*/
		/* need to retransmit the frame for which we were expecting	a reply.*/
		if (expected_frame_type != WlanC_None)
			{				
			/* Since the station did not receive the expected frame it has	*/
			/* to retransmit the packet. Increment the appropriate retry	*/
			/* count.														*/
			if (wlan_flags->pcf_active == OPC_TRUE) 
				{
				pcf_retry_count++;
				}
			else
				{
				/* Increment the short retry count if the transmission of	*/
				/* an RTS frame or a data frame smaller than RTS/CTS		*/
				/* threshold has failed; otherwise increment the long retry	*/
				/* count.													*/
				if (last_frametx_type == WlanC_Rts || wlan_flags->frame_size_req_rts == OPC_FALSE)
					short_retry_count++;
				else
					long_retry_count++;
				
				/* Also reset the rts_sent flag since we will recontend.	*/
				wlan_flags->rts_sent = OPC_FALSE;
				}
				
			/* Reset the NAV duration so that the retransmission is not		*/
			/* unnecessarily delayed.										*/
			if (cfp_ap_medium_control == OPC_FALSE)
				nav_duration = current_time;
			}
		
		/* Update NAV if necessary.											*/
		if (nav_duration < current_time + rcvd_duration)
			{
			nav_duration = current_time + rcvd_duration;				
			wlan_flags->nav_updated = OPC_TRUE;
			}					
		}
	else	
		{
		/* Unknown frame type so declare error.								*/
		wlan_error_print ("Unexpected frame type received.", OPC_NIL, OPC_NIL);
		}
   
	/* Check whether further retries are possible or	*/
	/* the data frame needs to be discarded.			*/
	if (wlan_flags->pcf_active)
		wlan_pcf_frame_discard ();
	else
		wlan_frame_discard ();

	/* Set the expected frame type to None because either the 	*/
	/* expected frame is received or the station will have to 	*/
	/* retransmit the frame										*/
	expected_frame_type = WlanC_None;
	
	/* Destroying the received frame once relevant information is taken out of it.	*/
	op_pk_destroy (wlan_rcvd_frame_ptr);														

	FOUT;
	}


static Boolean
wlan_tuple_find (OpT_Int64 sta_addr, int seq_cntl, OpT_Int64 dest_addr, Boolean retry)
	{
	WlanT_Peer_Info*	src_sta_info_ptr;

	/** This function checks whether the last received frame is a		**/
	/** duplicate or not using the source address, sequence number, and	**/
	/** fragmentation number information of the frame. It returns the	**/
	/** result of this check.											**/
	FIN (wlan_tuple_find (sta_addr, seq_cntl, dest_addr, retry));

	/* We receive packets only from our AP if we belong to an			*/
	/* infrastructure BSS, in which case we don't maintain a Hash table	*/
	/* for peer records.												*/
	if (peer_info_hash_tbl != OPC_NIL)
		{
		/* First make sure that the source STA belongs to our BSS.		*/
		src_sta_info_ptr = (WlanT_Peer_Info *) prg_bin_hash_table_item_get (peer_info_hash_tbl, (void *) &(sta_addr));
		if (src_sta_info_ptr != OPC_NIL)
			{
			/* Check whether the latest received frame is a duplicate.	*/
			if (retry == OPC_TRUE && src_sta_info_ptr->seq_cntl >> WLANC_FRAG_NUM_SIZE == seq_cntl >> WLANC_FRAG_NUM_SIZE && 
				src_sta_info_ptr->seq_cntl >= seq_cntl)
				duplicate_entry = OPC_TRUE;
			else
				{
				duplicate_entry = OPC_FALSE;
				
				/* Since this is not a duplicate, update the recorded	*/
				/* information for this source.							*/
				src_sta_info_ptr->seq_cntl  = seq_cntl;
				}
			
			/* Return the result.										*/
			FRET (duplicate_entry);
			}
		}
	else
		{
		/* Make sure that the frame is coming from our AP.				*/
		if (sta_addr == ap_mac_address)
			{
			/* Check whether the latest received frame is a duplicate.	*/
			if (retry == OPC_TRUE && ap_peer_info_ptr->seq_cntl >> WLANC_FRAG_NUM_SIZE == seq_cntl >> WLANC_FRAG_NUM_SIZE && 
				ap_peer_info_ptr->seq_cntl >= seq_cntl)
				duplicate_entry = OPC_TRUE;
			else
				{
				duplicate_entry = OPC_FALSE;
				
				/* Since this is not a duplicate, update the recorded	*/
				/* information for this source.							*/
				ap_peer_info_ptr->seq_cntl = seq_cntl;
				}
			
			/* Return the result.										*/
			FRET (duplicate_entry);
			}
		}
	
	/* If we come to this point, then either the source STA is not in	*/
	/* our BSS or we received a direct transmission from a peer			*/
	/* bypassing the AP in the BSS. In either case, this is an			*/
	/* unexpected event indicating an error condition, unless the		*/
	/* packet is a broadcast packet.									*/
	if (dest_addr == my_address)
		{
		wlan_error_print ("Receiving packet from a station that does not exist in this BSS",
						  "Possibly wrong destination address", "Please check the configuration");
		}
	else
		{
		/* We may receive broadcast packets that are not originated in	*/
		/* our BSS, if there are other BSSs in the vicinity	that uses	*/
		/* the exact same physical channel with our own	BSS. In such	*/
		/* cases, return OPC_TRUE, like we received a duplicate			*/
		/* message, so that the caller function destroys the packet		*/
		/* without any processing.										*/
		FRET (OPC_TRUE);
		}

	/* This line won't be reached.										*/
	FRET (OPC_TRUE);				
	}
	
static void
wlan_data_process (Packet* seg_pkptr, OpT_Int64 dest_addr, OpT_Int64 sta_addr, OpT_Int64 final_dest_addr, int protocol_type,
				int frag_num, int more_frag, OpT_Packet_Id pkt_id)
	{
	char										msg_string [120];
	int	 										current_index;
	int 										list_index;
	int 										list_size;
	Packet*										copy_pkptr;
	Boolean										send_to_higher;
	OpT_Packet_Size								pkt_size;
	WlanT_Mac_Defragmentation_Buffer_Entry*     defrag_ptr = OPC_NIL;
	
	/** This routine handles defragmentation process and also sends		**/
	/** data to the higher layer if all the fragments have been			**/
	/** received by the station.										**/
	FIN (wlan_data_process (seg_pkptr, dest_addr, sta_addr, final_dest_addr, protocol_type, frag_num, more_frag, pkt_id));

	/* First check for the case where the received segment contains the	*/
	/* entire data packet, i.e. the data is transmitted as a single		*/
	/* fragment.														*/
	if (frag_num == 0 && more_frag == 0)
		{
		/* Insert the segment into our "common" reassembly buffer that	*/
		/* is used for such single-fragment transmission.				*/
		op_sar_rsmbuf_seg_insert (common_rsmbuf_ptr, seg_pkptr);
		}
	
	else
		{
		/* The original packet is being transmitted in multiple			*/
		/* fragments. Insert fragments into the reassembly buffer.		*/
		/* There are two possible cases:								*/
		/* 1. The remote station has just started sending the 			*/
		/* fragments and it doesn't exist in the list.					*/
		/* 2. The remote station does exist in the list and the 		*/
		/* and the new fragment is a series of fragments for the data 	*/
		/* packet.													  	*/

		/* Get the size of the defragmentation list.					*/
		list_size = op_prg_list_size (defragmentation_list_ptr);

		/* Initialize the current node index which will indicate		*/
		/* whether the entry for the station exists in the list.		*/
		current_index = -1;

		/* Searching through the list to find if the remote station		*/
		/* address exists i.e. the source station has received			*/
		/* fragments for this data packet before. Also, removing		*/
		/* entries from the defragmentation buffer which has reached	*/
		/* its maximum receive lifetime.								*/
		for (list_index = 0; list_index < list_size; )
			{
			/* Accessing node of the list for search purposes.			*/						
			defrag_ptr = (WlanT_Mac_Defragmentation_Buffer_Entry *) op_prg_list_access (defragmentation_list_ptr, list_index);

			/* Removing station entry if the receive lifetime has		*/
			/* expired.													*/
			if ((current_time - defrag_ptr->time_rcvd) >= max_receive_lifetime)
				{
				/* Removing the partially completed fragment once its	*/
				/* lifetime has reached.								*/
				defrag_ptr = (WlanT_Mac_Defragmentation_Buffer_Entry *) op_prg_list_remove (defragmentation_list_ptr, list_index);
				op_sar_buf_destroy (defrag_ptr->reassembly_buffer_ptr);					
				op_prg_mem_free (defrag_ptr);	

				/* Updating the total list size.						*/
				list_size = list_size - 1;
				}

			/* If the station entry already exists in the list then		*/
			/* store its index for future use.							*/
			else if (sta_addr == defrag_ptr->tx_station_address)
				{
				current_index = list_index;
			
				/* Exit the loop since we have found the entry we were	*/
				/* looking for.											*/
				list_index = list_size;	
				}
		
			/* Otherwise move to the next element in the list.			*/
			else
				list_index++;
			}                             						

		/* If remote station entry doesn't exist then create new node.	*/
		if (current_index == -1)											
			{
			/* If the entry of the station does not exist in the defrag	*/
			/* list and the fragment received is not the first fragment	*/
			/* of the packet then it implies that the maximum receive	*/
			/* lifetime of the packet has expired. In this case the		*/
			/* received packet will be destroyed and the				*/
			/* acknowledgement is sent to the receiver as specified by	*/
			/* the protocol.											*/
			if (frag_num > 0)
				{
				op_pk_destroy (seg_pkptr);
				FOUT;
				}

			/* Creating struct type for defragmentation structure.		*/  						
			defrag_ptr = (WlanT_Mac_Defragmentation_Buffer_Entry *) op_prg_mem_alloc (sizeof (WlanT_Mac_Defragmentation_Buffer_Entry));

			/* Generate error and abort simulation if no more memory	*/
			/* left to allocate for duplicate buffer.					*/
			if (defrag_ptr == OPC_NIL)
				{
				wlan_error_print ("Cannot allocate memory for defragmentation buffer entry", OPC_NIL, OPC_NIL);
				}	

			/* Source station address is store in the list for future	*/
			/* reference.												*/
			defrag_ptr->tx_station_address = sta_addr;

			/* For new node creating a reassembly buffer.				*/
			defrag_ptr->reassembly_buffer_ptr = op_sar_buf_create (OPC_SAR_BUF_TYPE_REASSEMBLY, OPC_SAR_BUF_OPT_DEFAULT);
			op_prg_list_insert (defragmentation_list_ptr, defrag_ptr, OPC_LISTPOS_TAIL);
			}

		else if (frag_num == 0)
			{
			/* We have found a defragmentation buffer for the source	*/
			/* station, although this is the first fragment of a		*/
			/* transmission sequence. This is a rare case and indicates	*/
			/* that we were receiving another fragmented MSDU from the	*/
			/* same source, which the source completed sending though	*/
			/* we could not receive all the fragments. Hence, flush the	*/
			/* defragmentation buffer since we will never receive the	*/
			/* missing fragment(s) of the MSDU that is already in the	*/
			/* buffer.													*/
			op_sar_rsmbuf_pk_flush (defrag_ptr->reassembly_buffer_ptr, 0.0);
			}
		
		/* Record the received time of this fragment.					*/
		defrag_ptr->time_rcvd = current_time;
					
		/* Insert fragment into the reassembly buffer.					*/
		op_sar_rsmbuf_seg_insert (defrag_ptr->reassembly_buffer_ptr, seg_pkptr);
		}

	/* If this is the last fragment then send the data to higher layer.	*/
	if (more_frag == 0)
		{
		/* Pick the correct reassembly buffer based on the fragment		*/
		/* type.														*/
		if (frag_num > 0)
			seg_pkptr = op_sar_rsmbuf_pk_remove (defrag_ptr->reassembly_buffer_ptr);
		else
			seg_pkptr = op_sar_rsmbuf_pk_remove (common_rsmbuf_ptr);
		
		/* Make sure that we really have all the fragments. Otherwise	*/
		/* it indicates an internal operational error probably at the	*/
		/* source node.													*/
		if (seg_pkptr == OPC_NIL)
			wlan_error_print ("Reassembly error at the destination MAC.", "Probably, source MAC sent an MPDU with empty payload.", OPC_NIL);
		
		if (ap_flag == OPC_BOOLINT_ENABLED) 
			{
			/* If the address is not found in the address list then		*/
			/* access point will sent the data to higher layer for		*/
			/* address resolution. Note that if destination address is	*/
			/* same as AP's address then the packet is sent to higher	*/
			/* layer for address resolution. If the destination address	*/
			/* is broadcast address then the packet is both transmitted	*/
			/* within the BSS and also forwarded to the	higher layer.	*/	
			if (final_dest_addr == MAC_BROADCAST_ADDR ||
				(final_dest_addr != my_address && prg_bin_hash_table_item_get (peer_info_hash_tbl, (void *) &(final_dest_addr)) != PRGC_NIL))
				{
				/* Printing out information to ODB.						*/
				if (wlan_trace_active == OPC_TRUE)
					{
					sprintf (msg_string, "All fragments of data packet " OPC_PACKET_ID_FMT " is received by the AP", pkt_id);
					op_prg_odb_print_major (msg_string, "and enqueued for transmission within the BSS.", OPC_NIL);
					}

				/* If the destination address is broadcast address then	*/
				/* we need to send a copy also to the higher layer.		*/
				if (final_dest_addr == MAC_BROADCAST_ADDR)
					{
					copy_pkptr = op_pk_copy (seg_pkptr);
					send_to_higher = OPC_TRUE;
					}
				else
					{
					copy_pkptr = seg_pkptr;
					send_to_higher = OPC_FALSE;
					}
				  
				/* Enqueuing packet for transmission within our BSS.	*/
				/* First check whether we have sufficient buffer space	*/
				/* to store the packet.									*/
				pkt_size = op_pk_total_size_get (copy_pkptr);
				if (total_hlpk_size + pkt_size > hld_max_size)
					{
					/* Buffer is too full to accept the packet.			*/
					wlan_hl_packet_drop (copy_pkptr, pkt_size);
					}
				else
					{
					/* Update the buffer size information and enqueue	*/
					/* the packet.										*/
					total_hlpk_num++;
					total_hlpk_size += pkt_size;
					if (wlan_poll_list_member_find (final_dest_addr) == OPC_TRUE)
						{
						wlan_hlpk_enqueue (copy_pkptr, final_dest_addr, sta_addr, protocol_type, pkt_size, OPC_TRUE);
						}
					else
						{
						wlan_hlpk_enqueue (copy_pkptr, final_dest_addr, sta_addr, protocol_type, pkt_size, OPC_FALSE);
						wlan_flags->data_frame_to_send  = OPC_TRUE;
						}
					}
				}
			else
				send_to_higher = OPC_TRUE;
			
			/* Send the packet to the higher layer if not destined		*/
			/* within own BSS or if it has broadcast address as			*/
			/* destination address.										*/
			if (send_to_higher == OPC_TRUE)
				{				
				/* Update the local/global throughput and end-to-end	*/
				/* delay statistics based on the packet that will be	*/
				/* forwarded to the higher layer.						*/
				wlan_accepted_frame_stats_update (seg_pkptr);

				/* Set the contents of the LLC-destined ICI -- set the	*/
				/* address of the transmitting station.					*/
				if (op_ici_attr_set_int64 (llc_iciptr, "src_addr", sta_addr) == OPC_COMPCODE_FAILURE)
					{
					wlan_error_print ("Unable to set source address in LLC ICI.", OPC_NIL, OPC_NIL);
					}

				/* Set the destination address (this mainly serves to	*/
				/* distinguish packets received under broadcast			*/
				/* conditions.)											*/
				if (op_ici_attr_set_int64 (llc_iciptr, "dest_addr", final_dest_addr) == OPC_COMPCODE_FAILURE)
					{
					wlan_error_print("Unable to set destination address in LLC ICI.", OPC_NIL, OPC_NIL);
					}
		
				/* Set the protocol type field contained in the WLAN	*/
				/* frame.												*/
				if (op_ici_attr_set (llc_iciptr, "protocol_type", protocol_type) == OPC_COMPCODE_FAILURE)
					{
					wlan_error_print("Unable to set protocol type in LLC ICI.", OPC_NIL, OPC_NIL);
					}				

				/* Printing out information to ODB.						*/
				if (wlan_trace_active == OPC_TRUE)
					{
					sprintf (msg_string, "All fragments of Data packet " OPC_PACKET_ID_FMT " is received and sent to the higher layer", pkt_id);
					op_prg_odb_print_major (msg_string, OPC_NIL);
					}
				
				/* Setting an ici for the higher layer.					*/
				op_ici_install (llc_iciptr);

				/* Sending data to higher layer.						*/
				wlan_rcvd_pkt_higher_layer_forward (seg_pkptr, wlan_flags->bridge_flag, mac_client_reassembly_buffer, outstrm_to_mac_if);
				}
			}
		else
			{
			/* If the station is a gateway and not an access point then do not send		*/
			/* data to higher layer for address resolution.  This is for not allowing   */
			/* data to go out of the ad-hoc BSS. Except, in the case of broadcast		*/
			/* packets and packets addressed to this station. On the other hand, if we	*/
			/* are in a bridge/switch node and not AP enabled, then drop the packet.	*/
			if ((wlan_flags->gateway_flag == OPC_TRUE && dest_addr != my_address && dest_addr >= 0) || 
				wlan_flags->bridge_flag == OPC_TRUE)
				{				
				/* Printing out information to ODB.						*/
				if (wlan_trace_active == OPC_TRUE)
					{
					strcpy (msg_string, "Gateway is not an access point so all received fragments are discarded.");
					op_prg_odb_print_major (msg_string, OPC_NIL);
					}
				op_pk_destroy (seg_pkptr);
				}
			else
				{
				/* Update the local/global throughput and end-to-end	*/
				/* delay statistics based on the packet that will be	*/
				/* forwarded to the higher layer.						*/
				wlan_accepted_frame_stats_update (seg_pkptr);
				
				/* Send the packet to the higher layer unless it is a	*/
				/* spanning tree BPDU. No need to check whether the		*/
				/* surrounding node	is a bridge/switch since WLAN ports	*/
				/* can't be used for bridge-to-bridge connections.		*/
				if (dest_addr == BRIDGE_BROADCAST_ADDR || dest_addr == PVST_BPE_MCAST_ADDR)
					op_pk_destroy (seg_pkptr);
				else
					{
					/* Printing out information to ODB.					*/
					if (wlan_trace_active == OPC_TRUE)
						{
						sprintf (msg_string, "All fragments of Data packet " OPC_PACKET_ID_FMT " is received and sent to the higher layer", pkt_id);
						op_prg_odb_print_major (msg_string, OPC_NIL);
						}

					/* Sending data to the higher layer.				*/
					wlan_rcvd_pkt_higher_layer_forward (seg_pkptr, wlan_flags->bridge_flag, mac_client_reassembly_buffer, outstrm_to_mac_if);
					}
				}
			}
			
		/* If any used, destroyed any defragmentation entry and			*/
		/* reassembly buffer, since the fragments of frame were			*/
		/* completed and the data was sent to the higher layer.			*/
		if (defrag_ptr != OPC_NIL)
			{
			defrag_ptr = (WlanT_Mac_Defragmentation_Buffer_Entry *) op_prg_list_remove (defragmentation_list_ptr, current_index);
			op_sar_buf_destroy (defrag_ptr->reassembly_buffer_ptr);					
			op_prg_mem_free (defrag_ptr);
			}
		}
	else
		{
		/* Printing out information to ODB.	*/
		if (wlan_trace_active == OPC_TRUE)
			{			
			sprintf (msg_string, "Data packet " OPC_PACKET_ID_FMT " is received and waiting for more fragments ", pkt_id);
			op_prg_odb_print_major (msg_string, OPC_NIL);
			}
		}

	FOUT;
	}

static void
wlan_accepted_frame_stats_update (Packet* seg_pkptr)
	{
	double	ete_delay, pk_size;
	
	/** This function is called just before a frame received from	**/
	/** physical layer being forwarded to the higher layer to		**/
	/** update end-to-end delay and throughput statistics.			**/
	FIN (wlan_accepted_frame_stats_update (seg_pkptr));
	
	/* The packet has been accepted, tag the packet for apptrack receive */
	apptrack_rx_record (seg_pkptr);
	
	/* Total number of bits sent to higher layer is equivalent to a	*/
	/* throughput.													*/
	pk_size = (double) op_pk_total_size_get (seg_pkptr);
	op_stat_write (throughput_handle, pk_size);
	op_stat_write (throughput_handle, 0.0);

	/* Also update the global WLAN throughput statistic.			*/
	op_stat_write (global_throughput_handle, pk_size);
	op_stat_write (global_throughput_handle, 0.0);
	
	/* Compute the end-to-end delay for the frame and record it.	*/
	ete_delay = current_time - op_pk_stamp_time_get (seg_pkptr);
	op_stat_write (ete_delay_handle, 		ete_delay);
	op_stat_write (global_ete_delay_handle, ete_delay);
	
	FOUT;
	}


static void 
wlan_schedule_deference ()
	{
	/** This routine schedules self interrupt for deference **/
	/** to avoid collision and also deference to observe	**/
	/** interframe gap between the frame transmission.		**/
	FIN (wlan_schedule_deference ());

	/* Check the status of the receiver. If it is busy and the	*/
	/* frame to send is not ACK, exit the function, since we 	*/
	/* will schedule the end of the deference when it becomes 	*/
	/* idle (ACK over rides receiver busy signal (see 9.2.8)).	*/
	if (wlan_flags->receiver_busy == OPC_TRUE && fresp_to_send != WlanC_Ack)
		{
		FOUT;
		} 

	/* Extracting current time at each interrupt.				*/
	current_time = op_sim_time ();
	
	/* Adjust the NAV if necessary.								*/
	if (nav_duration < rcv_idle_time)
		{
		nav_duration = rcv_idle_time;
		}

	/* Just wait for SIFS if we need to send an ACK.			*/
	if (fresp_to_send == WlanC_Ack) 
		{
	
		/* Set deference interrupt for SIFS time.				*/
		deference_evh = op_intrpt_schedule_self (current_time + sifs_time, WlanC_Deference_Off);
		
		/* Disable backoff since not used with SIFS.			*/
		wlan_flags->backoff_flag = OPC_FALSE;
		wlan_flags->perform_cw   = OPC_FALSE;
		
		/* Set ignore busy flag to prevent canceling of			*/
		/* deference by receiver busy.							*/
		wlan_flags->ignore_busy = OPC_TRUE;
		}
	
	/* If it is time to send a Beacon frame and if this is going to	be	*/
	/* the beginning of a new CFP, use a PIFS time for deference.		*/
	/* Beacon will preempt any frame sequence unless we have an ACK to	*/
	/* send or we are in the middle of sending fragments of a			*/
	/* fragmented packet.												*/
	else if	(wlan_flags->tx_beacon == OPC_TRUE && wlan_flags->pcf_active == OPC_FALSE && fresp_to_send != WlanC_Ack &&  
		     (op_sar_buf_size (fragmentation_buffer_ptr) == 0 || short_retry_count + long_retry_count > 0 ||
			  op_sar_buf_size (fragmentation_buffer_ptr) == packet_size_dcf))
		{
		/* Check if PIFS deference already satisfied. If satisfied		*/
		/* start the transmission immediately. Else schedule PIFS based	*/
		/* on last medium idle time. Note that Beacon currently does	*/
		/* not defer to NAV.											*/
		if	(current_time > (rcv_idle_time + pifs_time))
			deference_evh = op_intrpt_schedule_self (current_time, WlanC_Deference_Off);
		else
			deference_evh = op_intrpt_schedule_self (rcv_idle_time + pifs_time, WlanC_Deference_Off);
		
		/* Since we are going to transmit a Beacon frame disable the	*/
		/* backoff and contention window flags.							*/
		wlan_flags->backoff_flag = OPC_FALSE;
		wlan_flags->perform_cw   = OPC_FALSE;
		}

	else if (wlan_flags->pcf_active == OPC_TRUE)
		{
		/* PCF is active, so follow PCF deference and backoff rules. 	*/

		/* Disable backoff because this is the CFP (backoff is actually optional)	*/
		/* Note that a mandatory periodic DIFS + backoff during the CFP is not 		*/
		/* implemented.  (See 9.3.3.2)												*/
		wlan_flags->backoff_flag = OPC_FALSE;		
		wlan_flags->perform_cw   = OPC_FALSE;

		if (wlan_flags->pcf_side_traf == OPC_TRUE)
			{
			/* If last frame was side traffic must defer for ACK as well. Side traffic		*/
			/* requires an Ack from a station other than the AP and the AP may not be 		*/
			/* able to hear the Ack so AP should defer till the Ack would be completed.		*/
			/* Note that allowing side traffic in an infrastructure network was voted out	*/
			/* of the 802.11 standard and should not be permitted. However, the standard	*/
			/* was never fully updated and the wording is currently contradictory on this	*/
			/* point.  The side traffic capability can be shut off in the MAC model			*/
			/* attributes.	To account for side traffic poll response will set the duration	*/
			/* field to account for a WLAN_ACK duration and SIFS.  This is in violation of	*/
			/* the protocol since the duration field should be set to 32768 (0).  However,	*/
			/* since this whole thing is forbidden anyway, some liberties may be taken.		*/
			/* The AP will update it's NAV, and automatically defer the right amount of 	*/
			/* time.																		*/
			deference_evh = op_intrpt_schedule_self (nav_duration + sifs_time , WlanC_Deference_Off);
			wlan_flags->pcf_side_traf = OPC_FALSE;
			}

		else if (ap_flag == OPC_BOOLINT_ENABLED && (poll_fail_count != 0 || pcf_retry_count != 0))
			{
			/* This is an AP and a transmission has failed. Unless we are at the		*/
			/* beginning of a new CFP, we have already waited for pifs_time when		*/
			/* waiting for response - don't need additional deference.					*/
			if (poll_fail_count != 0)
				deference_evh = op_intrpt_schedule_self (current_time, WlanC_Deference_Off);
			else
				deference_evh = op_intrpt_schedule_self (current_time + sifs_time, WlanC_Deference_Off);
			}
	    else
			{
			/* Normal PCF transmission so use a sifs_time.								*/
			deference_evh = op_intrpt_schedule_self (current_time + sifs_time, WlanC_Deference_Off);
			}

		/* Set ignore busy flag to prevent canceling of deference by receiver busy.		*/
		wlan_flags->ignore_busy = OPC_TRUE;
		}
	
	else 
		{
		/* DCF is active, so follow DCF deference and backoff rules. 	*/
	
		/* Station needs to wait SIFS duration before responding to any */	
		/* frame. Also, if Rts/Cts is enabled then the station needs	*/
		/* to wait for SIFS duration after acquiring the channel using	*/
		/* Rts/Cts exchange.						*/
		if ((fresp_to_send != WlanC_None) || (wlan_flags->rts_sent == OPC_TRUE))
			{ 
			deference_evh = op_intrpt_schedule_self (current_time + sifs_time, WlanC_Deference_Off);

			/* Disable backoff and CW flags because this frame is a response frame	*/
			/* to the previously received frame (this could be Ack or Cts).			*/
			wlan_flags->backoff_flag = OPC_FALSE;
			wlan_flags->perform_cw   = OPC_FALSE;
			}

		/* If more fragments to send then wait for SIFS duration and transmit. 		*/
		/* Station need to contend for the channel if one of the fragments is not	*/
		/* successfully transmitted.												*/
		else if ((short_retry_count + long_retry_count == 0) && (op_sar_buf_size (fragmentation_buffer_ptr) > 0))
			{
			/* Scheduling a self interrupt after SIFS duration.						*/
			deference_evh = op_intrpt_schedule_self (current_time + sifs_time, WlanC_Deference_Off);
		
			/* Disable backoff because the frame need to be transmitted after SIFS	*/
			/* duration. This frame is part of the fragment burst.					*/
			wlan_flags->backoff_flag = OPC_FALSE;			
			}
	
		/* If the last frame we received was corrupted, which indicates a			*/
		/* collision, then we need to defer for a longer period, i.e. for EIFS		*/
		/* period, which starts when the medium becomes idle.						*/
		else if (wlan_flags->wait_eifs_dur == OPC_TRUE)
			{
			/* EIFS is required. We need to use the larger of rcv_idle_time + EIFS	*/
			/* and NAV + DIFS since EIFS period starts when the receiver becomes	*/
			/* idle regardless of the status of virtual carrier sensing (section	*/
			/* 9.2.3.4).															*/
			if (rcv_idle_time + eifs_time >= nav_duration + difs_time)
				deference_evh = op_intrpt_schedule_self ((rcv_idle_time + eifs_time), WlanC_Deference_Off);
			else
				deference_evh = op_intrpt_schedule_self ((nav_duration + difs_time), WlanC_Deference_Off);

			/* After an EIFS period, a backoff is needed.							*/
			if (wlan_flags->cw_required == OPC_TRUE)
				wlan_flags->perform_cw = OPC_TRUE;
			else
				wlan_flags->backoff_flag = OPC_TRUE;
		
			/* Reset the EIFS flag.													*/
			wlan_flags->wait_eifs_dur = OPC_FALSE;
			}
	
		/* If we are in contention window period, which follows a successful		*/
		/* packet transmission, set the flag to trigger the backoff period.			*/
		else if ((wlan_flags->cw_required == OPC_TRUE) && 
			(wlan_flags->polled == OPC_FALSE))
			{
			wlan_flags->perform_cw = OPC_TRUE;
			
			/* We defer for NAV duration plus DIFS duration before resuming for CW	*/
			/* backoff period.														*/
			deference_evh = op_intrpt_schedule_self ((nav_duration + difs_time), WlanC_Deference_Off);		
			}

		else if (wlan_flags->polled == OPC_TRUE)
			{
			/* Station only awaits a SIFS duration before sending a response frame	*/
			/* for the poll message it received.									*/
			deference_evh = op_intrpt_schedule_self (current_time + sifs_time, WlanC_Deference_Off);

			/* Disable the backoffs since not needed if SIFS time being used.			*/
			wlan_flags->backoff_flag = OPC_FALSE;
			wlan_flags->perform_cw   = OPC_FALSE;

			/* Set ignore busy flag to prevent canceling of deference by receiver busy.	*/
			wlan_flags->ignore_busy = OPC_TRUE;
			}
		
		else
			{
			/* If the station needs to transmit or retransmit frame, it will */
			/* defer for NAV duration plus  DIFS duration and then backoff   */
			deference_evh = op_intrpt_schedule_self ((nav_duration + difs_time), WlanC_Deference_Off);		
		
			/* Before sending data frame or Rts backoff is needed. */
			wlan_flags->backoff_flag = OPC_TRUE;
			}
		}
	
	/* Reset the updated NAV flag, since as of now we scheduled a new	*/
	/* "end of deference" interrupt after the last update.				*/
	wlan_flags->nav_updated = OPC_FALSE;
	
	FOUT;
	}

static void
wlan_frame_discard ()
	{
	int seg_bufsize;
	Packet* seg_pkptr;
//自添加
	//$$$$$$$$$$$$$$$$$$ DSR $$$$$$$$$$$$$$$$$$$$$$$$
	Objid process_id;
	Ici* iciptr;
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//自添加结束
	
	/** No further retries for the data frame for which the retry limit has reached.	**/
	/** As a result these frames are discarded.											**/
	FIN (wlan_frame_discard ());

	/* If one of the retry limits was reached then drop the frame.	*/
	if (short_retry_count == short_retry_limit || long_retry_count == long_retry_limit)
		{
		/* Update retransmission count statistic. Subtract one from the		*/
		/* total of the retry counts since the last retransmission actually	*/
		/* didn't happen as a result of reaching the limit.					*/
		op_stat_write (retrans_handle, 			(double) (short_retry_count + long_retry_count - 1));
		op_stat_write (global_retrans_handle, 	(double) (short_retry_count + long_retry_count - 1));		

		/* Update the local and global statistics that record discarded		*/
		/* data traffic as a result of reaching retry limit.				*/
		op_stat_write (retx_drop_pkts_handle,   1.0);
		op_stat_write (retx_drop_pkts_handle,   0.0);
		op_stat_write (retx_drop_bits_handle,   (double) packet_size_dcf);
		op_stat_write (retx_drop_bits_handle,   0.0);
		op_stat_write (global_retx_drop_handle, (double) packet_size_dcf);
		op_stat_write (global_retx_drop_handle, 0.0);

		/* Update the queue size information and statistic.					*/
		total_hlpk_num--;
		total_hlpk_size -= packet_size_dcf;
		op_stat_write (hl_packets_rcvd, (double) total_hlpk_num);
//自添加
		//$$$$$$$$$$$$$$$$$$ DSR $$$$$$$$$$$$$$$$$$$$$$$$
		// DSR: Send an error intrpt to the DSR routing layer and collect statistic
		if (data_packet_type==DATA_PACKET_TYPE)
			{
			printf("--- MAC Msg @ node %d --- sending NACK for destination %d\n", my_address,data_packet_dest); 
			iciptr = op_ici_create("aodv_nack");
			op_ici_attr_set(iciptr,"Relay_Destination",data_packet_dest);    
			op_ici_attr_set(iciptr,"Final_Destination",data_packet_final_dest);
			op_ici_install(iciptr);
			process_id=op_id_from_name(op_topo_parent (op_id_self()),OPC_OBJTYPE_QUEUE,"aodv_routing");
			op_intrpt_schedule_remote(op_sim_time(),ERROR_CODE,process_id);
			}
		if (1)
			{
			op_prg_odb_bkpt("mac_failed");
			}
		//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//自添加结束
		/* Reset the retry counts for the next packet.	*/
		short_retry_count = 0;
		long_retry_count  = 0;
		
		/* Get the segmentation buffer size to check if there are more fragments left to be transmitted.	*/
		seg_bufsize =  (int) op_sar_buf_size (fragmentation_buffer_ptr); 

		if (seg_bufsize != 0)
			{
			/* Discard remaining fragments	*/
			seg_pkptr = op_sar_srcbuf_seg_remove (fragmentation_buffer_ptr, seg_bufsize);
			op_pk_destroy (seg_pkptr);
			}

		/* If expecting Ack frame then destroy the tx data frame as this frame will	*/
		/* no longer be transmitted (even if we are not expecting an Ack at this	*/
		/* moment, we still may have a copy of the frame if at one point in the		*/
		/* retransmission history of the original packet we received a Cts for our	*/
		/* Rts but then didn't receive an Ack for our data transmission; hence		*/
		/* consider this case as well).												*/
		if ((expected_frame_type == WlanC_Ack) || (wlan_transmit_frame_copy_ptr != OPC_NIL))
			{	
			/* Destroy the copy of the frame as the packet is discarded.	*/
			op_pk_destroy (wlan_transmit_frame_copy_ptr);
			wlan_transmit_frame_copy_ptr = OPC_NIL;
			}
			
		/* Reset the "frame to respond" variable unless we have a CTS or  	*/
		/* ACK to send.														*/
		if (fresp_to_send == WlanC_Data)
			{
			fresp_to_send = WlanC_None;
			}
		
		/* If there is not any other data packet sent from higher layer and	*/
		/* waiting in the buffer for transmission, reset the related flag.	*/
		if (op_prg_list_size (hld_list_ptr) == 0)
			{
			wlan_flags->data_frame_to_send = OPC_FALSE;
			}
		
		/* Although we could not transmit this data packet and eventually	*/
		/* dropped it, still set the contention window flag and back-off	*/
		/* for a contention window period. This is necessary for the		*/
		/* fairness of the algorithm. This prevents us going to IDLE state	*/
		/* (if higher layer data queue is empty) and then may attempt to	*/
		/* transmit a packet without waiting for a full backoff period as a	*/
		/* result of suddenly receiving a packet from higher layer.			*/
		wlan_flags->cw_required = OPC_TRUE;
		}
	
	FOUT;
	}

static void
wlan_pcf_frame_discard (void)
	{
	int 		seg_bufsize;
	Packet* 	seg_pkptr;
	
	/** No further retries for the data frame for which the retry limit has	**/
	/** reached. As a result these frames are discarded.					**/
	FIN (wlan_pcf_frame_discard ());

	/* If retry limit was reached then drop the frame. Assume DCF long		*/
	/* retry limit also the retry limit for PCF operation when no ACK is	*/
	/* received for	data transmissions.										*/
	if (pcf_retry_count == long_retry_limit)
		{
		/* Update retransmission count statistic.							*/						
		op_stat_write (retrans_handle, 			(double) (pcf_retry_count));
		op_stat_write (global_retrans_handle, 	(double) (pcf_retry_count));		

		/* Update the local and global statistics that record discarded		*/
		/* data traffic as a result of reaching retry limit.				*/
		op_stat_write (retx_drop_pkts_handle,   1.0);
		op_stat_write (retx_drop_pkts_handle,   0.0);
		op_stat_write (retx_drop_bits_handle,   (double) packet_size_pcf);
		op_stat_write (retx_drop_bits_handle,   0.0);
		op_stat_write (global_retx_drop_handle, (double) packet_size_pcf);
		op_stat_write (global_retx_drop_handle, 0.0);

		/* Update the queue size information and statistic.					*/
		total_hlpk_num--;
		total_hlpk_size -= packet_size_pcf;
		op_stat_write (hl_packets_rcvd, (double) total_hlpk_num);
		
		/* Reset the retry count for the next packet.						*/
		pcf_retry_count = 0;		
		
		/* Destroy the packet whose transmission trials kept failing.		*/
		if (pcf_frag_buffer_ptr != OPC_NIL)
			{			
			/* Get the segmentation buffer size to check if there are more	*/
			/* fragments left to be transmitted.							*/
			seg_bufsize =  (int) op_sar_buf_size (pcf_frag_buffer_ptr); 
			if (seg_bufsize != 0)
				{
				/* Discard remaining fragments.								*/
				seg_pkptr = op_sar_srcbuf_seg_remove (pcf_frag_buffer_ptr, seg_bufsize);
				
				if (seg_pkptr != OPC_NIL)
					op_pk_destroy (seg_pkptr);
				}

			/* If expecting ACK frame or too many polls, then destroy the 	*/
			/* tx data frame as this frame will no longer be transmitted.	*/
			if ((expected_frame_type == WlanC_Ack) || (poll_fail_count > max_poll_fails) ||
				(wlan_pcf_transmit_frame_copy_ptr != OPC_NIL))
				{	
				/* Destroy the copy of the frame as the packet is discarded.*/				
				if (wlan_pcf_transmit_frame_copy_ptr != OPC_NIL)
					{
					op_pk_destroy (wlan_pcf_transmit_frame_copy_ptr);
					}					
				wlan_pcf_transmit_frame_copy_ptr = OPC_NIL;
				}
			}

		/* Reset the counter for failed PCF polls.							*/
		poll_fail_count = 0;

		/* Reset the "frame to respond" variable unless we have a CTS or  	*/
		/* ACK to send.														*/
		if (fresp_to_send == WlanC_Data)
			{
			fresp_to_send = WlanC_None;
			}
		}
	
	FOUT;
	}

static void
wlan_mac_rcv_channel_status_update (int channel_id)
	{
	/** This function updates the receiver_busy flag based	**/
	/** on the the current value of the receiver's received	**/
	/** power statistic and reception end time state		**/
	/** information.										**/				
	FIN (wlan_mac_rcv_channel_status_update (int channel_id));
	
	/* Read the current value of the received power at the	*/
	/* receiver.											*/
	if (op_stat_local_read (channel_id) > rx_power_threshold)
		{
		/* This is the start of the reception of a new		*/
		/* packet. If the receiver was already busy and the	*/
		/* collision flag was not set, then set the			*/
		/* collision flag to true.							*/
		if (!wlan_flags->collision && wlan_flags->receiver_busy)
			wlan_flags->collision = OPC_TRUE;
		
		/* Set the receiver status as busy.					*/
		if (!wlan_flags->receiver_busy)
			{
			wlan_flags->receiver_busy = OPC_TRUE;
			
			/* Cancel the NAV reset interrupt, if any,		*/
			/* since we started receiving a packet.			*/
			if (op_ev_valid (nav_reset_evh))
				op_ev_cancel (nav_reset_evh);
			}
		}
	
	/* Else a packet reception is complete. Check whether	*/
	/* the receiver became available while it was busy. It	*/
	/* may not have been busy if we were receiving a noise	*/
	/* packet with a weak signal.							*/
	else if (wlan_flags->receiver_busy)
		{
		/* Compare the receiver's reception end time value	*/
		/* with the current time to determine its status.	*/
		if (rx_state_info_ptr->rx_end_time - PRECISION_RECOVERY <= current_time)
			{		
			wlan_flags->receiver_busy = OPC_FALSE;
			wlan_flags->collision = OPC_FALSE;			
			}
		
		/* Update the receiver idle time with current time.	*/
		/* Even though the receiver can be still busy (in	*/
		/* case of a collision), this is necessary to note	*/
		/* the completion time of the last reception.		*/
		rcv_idle_time = current_time;
		}
	
	FOUT;
	}


static Boolean
wlan_poll_list_member_find (OpT_Int64 dest_addr) 
	{
	int		i;

	/** This routine determines whether a given		**/
	/** address is on the polling list.				**/
	FIN (wlan_poll_list_member_find(dest_addr));

	/* if PCF not enabled or not an AP, then don't	*/
	/* use polling list.							*/
	if ((ap_flag == OPC_BOOLINT_DISABLED) || (pcf_flag == OPC_BOOLINT_DISABLED))
		{FRET (OPC_FALSE);}

	/* Send broadcast packets under DCF.			*/ 
	if (dest_addr < 0) {FRET (OPC_FALSE);}

	/* Otherwise, check if address on polling list.	*/
	for (i = 0; i < poll_list_size; i++ )
		{
		if (dest_addr == polling_list [i]) {FRET (OPC_TRUE);}
		if (dest_addr < polling_list [i]) {FRET (OPC_FALSE);}
		}

	/* Destination address is not on polling list.	*/ 
	FRET (OPC_FALSE);				
	}

static void
wlan_sta_addr_register (int bssid, OpT_Int64 sta_addr, int sta_is_ap, Objid sta_mac_objid, WlanT_Phy_Char_Code phy_char)
	{ 
	PrgT_Mapping_Handle 	wlan_bss_mapping_hndl;
	WlanT_Bss_Mapping_Info*	bss_info_ptr;
	char*					reduced_subnet_hname;
	char					stat_info [32];
	char					subnet_name [128];

	/** Add the station information to the BSS-based station map.			**/
	FIN (wlan_sta_addr_register (bssid, sta_addr, sta_is_ap, sta_mac_objid, phy_char));

	/* Get the BSS mapping handle (owned by called function as a static		*/
	/* variable).															*/
	wlan_bss_mapping_hndl = wlan_bss_mapping_get ();

	/* Check if this is the first station being mapping into this BSS.		*/
	bss_info_ptr = (WlanT_Bss_Mapping_Info *) prg_mapping_value_get (wlan_bss_mapping_hndl, &bssid);
	if (bss_info_ptr == PRGC_NIL)
		{
		/* Create the bss information record and store it in the BSS		*/
		/* mapping.															*/
		bss_info_ptr = (WlanT_Bss_Mapping_Info *) op_prg_mem_alloc (sizeof (WlanT_Bss_Mapping_Info));
		bss_info_ptr->bss_idx              = bssid;
		bss_info_ptr->non_erp_sta_count    = 0;
		bss_info_ptr->nqsta_count   	   = 0;
		bss_info_ptr->ap_objid 			   = OPC_OBJID_INVALID;
		bss_info_ptr->ap_dist_edca_params  = OPC_FALSE;
		bss_info_ptr->is_nqbss			   = OPC_TRUE;
		bss_info_ptr->ap_block_ack_support = OPC_FALSE;
		prg_mapping_value_add (wlan_bss_mapping_hndl, bss_info_ptr);

		/* Annotate the global network load statistic, which is collected	*/
		/* separately for each BSS, here, so that we prevent redundant		*/
		/* repetition of the same	task by the other members of the BSS.	*/
		/* If the BSS IDs are auto-assigned then use the name of the subnet	*/
		/* as the annotation, otherwise use the BSS ID.						*/
		if (bss_id_type == WlanC_Entire_Subnet)
			{
			/* Get the name of the subnet, where the nodes of the BSS		*/
			/* reside.														*/
			op_ima_obj_attr_get_str (my_subnet_objid, "name", 128, subnet_name);
			
			/* If the subnet name is not unique, then use the hierarchical	*/
			/* name.														*/
			if (oms_tan_is_subnet_name_unique (subnet_name) == OPC_FALSE)
				{
				op_ima_obj_hname_get (my_subnet_objid, subnet_name, 128);
				
				/* Skip the "top." prefix of the hierarchical name.			*/
				reduced_subnet_hname = &(subnet_name [4]);
				
				/* Use the reduced name for annotation.						*/
				Oms_Dim_Stat_Annotate (global_network_load_handle, reduced_subnet_hname);
				}
			else
				Oms_Dim_Stat_Annotate (global_network_load_handle, subnet_name);				
			}
		else
			{
			sprintf (stat_info, "BSS %d", bssid);
			Oms_Dim_Stat_Annotate (global_network_load_handle, stat_info);
			}
		}

	/* Save the MAC address and module object ID of the AP for the given	*/
	/* BSS.																	*/
	if (sta_is_ap == OPC_BOOLINT_ENABLED)
		{
		bss_info_ptr->ap_sta_addr = sta_addr;
		bss_info_ptr->ap_objid    = sta_mac_objid;
		bss_info_ptr->ap_phy_char = phy_char;
		bss_info_ptr->ap_prhndl	  = op_pro_self ();
		}

	/* Increment the count of 802.11/11b running STAs in the BSS if we are	*/
	/* one of them.															*/
	if (phy_type == WlanC_11b_PHY)
		{
		bss_info_ptr->non_erp_sta_count++;
	
		/* If this is not the association performed at initialization and	*/
		/* if we are the first non-ERP STA in the BSS and if the beacon		*/
		/* efficiency mode is enabled, then send a remote interrupt to the	*/
		/* AP of the BSS, so that it can inform the other STAs in the BSS	*/
		/* about the existence of a non-ERP STA in the LAN.					*/
		if (bss_info_ptr->non_erp_sta_count == 1 && BEACON_TX_EFFICIENCY_ENABLED && current_time > 0.0)
			{
			op_intrpt_schedule_remote (current_time, WlanC_Beacon_Tx_Time, bss_info_ptr->ap_objid);
			}
		}
	
	/* If this is the initial registration at time zero, then increment		*/
	/* the number of nQSTAs in our BSS, since we are one of the. Later on	*/
	/* this information will be maintained by the AP of the BSS, if needed.	*/
	if (current_time == 0.0)
		bss_info_ptr->nqsta_count++;
	
	FOUT;
	}

static void
wlan_sta_addr_deregister (int bssid, OpT_Int64 sta_addr)
	{
	PrgT_Mapping_Handle		wlan_bss_mapping_hndl;
	WlanT_Bss_Mapping_Info*	bss_info_ptr;
	
	/** This function removes the given STA address from the STA list of	**/
	/** the given BSS.														**/
	FIN (wlan_sta_addr_deregister (bssid, sta_addr));

	/* Get a handle to the mapping between BSS ID and BSS information.		*/
	wlan_bss_mapping_hndl = wlan_bss_mapping_get ();

	/* Access the BSS information.											*/
	bss_info_ptr = (WlanT_Bss_Mapping_Info *) prg_mapping_value_get (wlan_bss_mapping_hndl, &bssid);

	/* Decrement the count of 802.11/11b running STAs in the BSS if we were	*/
	/* one of them.															*/
	if (phy_type == WlanC_11b_PHY)
		{
		bss_info_ptr->non_erp_sta_count--;
	
		/* If we were the last non-ERP STA left in the BSS and if the		*/
		/* beacon efficiency mode is enabled, then send a remote interrupt	*/
		/* to the AP of the BSS, so that it can inform the other STAs in	*/
		/* the BSS about the departure of all non-ERP STAs from the LAN.	*/
		if (bss_info_ptr->non_erp_sta_count == 0 && BEACON_TX_EFFICIENCY_ENABLED)
			{
			op_intrpt_schedule_remote (current_time, WlanC_Beacon_Tx_Time, bss_info_ptr->ap_objid);
			}
		}
	
	/* Inform the AP about our disassociation so that it can update its		*/
	/* tables.																*/
	op_intrpt_schedule_process (bss_info_ptr->ap_prhndl, current_time, 
		((int) sta_addr << WLANC_ADDRESS_BIT_SHIFT) + ((phy_char_flag == WlanC_ERP_OFDM_11g) ? WLANC_ERP_SUPPORT_BIT : 0));

	FOUT;
	}

static void
wlan_begin_new_scan (void)
	{
	/** This function switches the node's communication channel to a new	**/
	/** one at least for a short period in order to evaluate the AP of this	**/
	/** channel, if any, for possible connection.							**/
	FIN (wlan_begin_new_scan (void));
	
	/* Set the AP Connectivity stat to -1 if we lost connectivity with		*/
	/* previous AP and just started the scanning procedure.					*/
	if (!(intrpt_type == OPC_INTRPT_SELF && intrpt_code == WlanC_Scan_Timeout))
		op_stat_write (ap_conn_handle, WLANC_AP_UNCONNECTED);
	
	/* Pick new channel so that it is five channels away-- the next 		*/
	/* non-overlapping channel (we first subtract one and then add one		*/
	/* because channel numbers start with 1 not 0).							*/
	channel_num = (channel_num - 1 + WLANC_CH_STEP_FOR_NO_OVERLAP) % channel_count + 1;
	
	/* If we are going to evaluate the new channel by listening to the		*/
	/* Beacons of its AP, configure our transceivers accordingly.			*/
	if (roam_state_ptr->scan_type == WlanC_Scan_Type_Beacon)
		{
		/* Set the transmitter and receiver to this channel.				*/
		wlan_set_transceiver_channel (channel_num, phy_type, txch_objid, rxch_objid);

		/* Initialize the BSS ID and reliability of the AP in this new		*/
		/* channel.															*/
		roam_state_ptr->ap_reliability = WLANC_AP_RELIABILITY_UNKNOWN;
		eval_bss_id = WLANC_BSS_ID_UNKNOWN;
		}
		
	/* Set a timer for slightly over two beacon periods, which will			*/
	/* indicate the end of the evaluation period of the new channel.		*/
	op_intrpt_schedule_self (current_time + beacon_int * WLANC_NEW_SCAN_BEACON_MULT, WlanC_Scan_Timeout);
	
	FOUT;
	}

static void
wlan_ap_switch (void)
	{
	int		i;
	char	stat_info [16];

	/** The function performs switching from old BSS/AP to new BSS/AP.		**/
	FIN (wlan_ap_switch (void));

	/* First lock the mutex that manages accessing the global mapping		*/
	/* information between the APs and stations, since we will be updating	*/
	/* this information.													*/
	op_prg_mt_mutex_lock (mapping_info_mutex, OPC_MT_MUTEX_LOCK_WRITER);
	
	/* Remove this STA from the STA list of the old BSS.					*/
	wlan_sta_addr_deregister (bss_id, my_address);

	/* Update own BSS ID information.										*/
	bss_id = eval_bss_id;
	my_bss_info_ptr = wlan_bss_info_get (bss_id);
	
	/* Also update roaming state for later comparison while evaluating APs.	*/
	roam_state_ptr->current_bss_id = bss_id;
	
	/* Find out the MAC address of our new AP.								*/
	ap_mac_address = wlan_get_ap_sta_addr (bss_id);

	/* Reset the peer information we maintain for the AP we are connected.	*/
	ap_peer_info_ptr->seq_cntl = 0xFFFF;

	/* Add ourselves to the STA list of our new BSS.						*/
	wlan_sta_addr_register (bss_id, my_address, OPC_BOOLINT_DISABLED, OPC_OBJID_INVALID, phy_char_flag);
	
	/* Update our handle for the network load statistic, which is			*/
	/* dimensioned with respect to BSSs, since we belong to a new BSS now.	*/
	sprintf (stat_info, "%d", bss_id);
	global_network_load_handle = Oms_Dim_Stat_Reg (my_objid, "Wireless LAN", "Network Load (bits/sec)", stat_info, OPC_STAT_GLOBAL);

	/* Determine the WLAN technology used by the new AP.					*/
	ap_peer_info_ptr->is_erp = (my_bss_info_ptr->ap_phy_char == WlanC_ERP_OFDM_11g) ? OPC_TRUE : OPC_FALSE;
	
	/* If we are an ERP-STA, then check whether there are any non-ERP STAs	*/
	/* in our new BSS.														*/
	if (phy_type == WlanC_11g_PHY)
		{
		/* Pick the value of the CWmin parameter based on whether the new	*/
		/* AP supports ERP (802.11g) operation and data rates. We may need	*/
		/* to adjust also our data transmission rate if the new AP doesn't	*/
		/* support 11g data rates.											*/	
		if (ap_peer_info_ptr->is_erp == OPC_TRUE)
			{
			/* The new AP also operates in 11g mode.						*/
			cw_min = 15;
			operational_speed = data_tx_rate;
			}
		else
			{
			/* The new AP doesn't support 11g operation. Set the CWmin to	*/
			/* 31 and lower the transmission rate to an 11b data rate.		*/ 
			cw_min = 31;
			if (data_tx_rate >= 11000000.0)
				operational_speed = 11000000.0;
			else if (data_tx_rate > 5500000.0)
				operational_speed = 5500000.0;
			}

		/* Compare the value of our non_erp_present flag with the situation	*/
		/* in our new BSS.													*/
		if (wlan_flags->non_erp_present == OPC_FALSE && my_bss_info_ptr->non_erp_sta_count > 0)
			{
			/* We have non-ERP STA(s) in our new BSS. Set the flag.			*/
			wlan_flags->non_erp_present = OPC_TRUE;
			
			/* Increase the slot time to 20 usec and recompute the			*/
			/* dependent parameters.										*/
			wlan_slot_time_set (20E-06);
			
			/* Reduce the control frame data rate to 802.11/11b mandatory	*/
			/* data rate.													*/
			control_data_rate = WLANC_11b_MIN_MANDATORY_DRATE;
			}
		
		else if (wlan_flags->non_erp_present == OPC_TRUE && my_bss_info_ptr->non_erp_sta_count == 0)
			{
			/* The new BSS doesn't have any non-ERP STA. Reset the flag.	*/
			wlan_flags->non_erp_present = OPC_FALSE;
			
			/* Decrease the slot time to 9 usec and recompute the dependent	*/
			/* parameters.													*/
			wlan_slot_time_set (9E-06);
			
			/* Reselect the control frame data rate. Choose the highest		*/
			/* mandatory data rate that is equal to or lower than the data	*/
			/* rate specified for data transmissions.						*/
			for (i = 0; data_tx_rate < WLANC_11g_MANDATORY_DRATE_ARRAY [i]; i++);
			control_data_rate = WLANC_11g_MANDATORY_DRATE_ARRAY [i];
			}		
		}
	
	/* Unlock the mutex that we locked at the beginning.					*/
	op_prg_mt_mutex_unlock (mapping_info_mutex);

	/* Inform the AP about our association with the BSS so that it can 		*/
	/* update its own tables.												*/
	op_intrpt_schedule_process (my_bss_info_ptr->ap_prhndl, current_time, ((int) my_address << WLANC_ADDRESS_BIT_SHIFT) + 
		WLANC_ASSOCIATION_BIT + ((phy_char_flag == WlanC_ERP_OFDM_11g) ? WLANC_ERP_SUPPORT_BIT : 0));
		
	FOUT;
	}

static void
wlan_reset_sv (void)
	{
	/** This function is called when the STA joins a new BSS. The state of	**/
	/** the MAC must be reset to ensure that old state does not disrupt the	**/
	/** correct operation of the MAC protocol. 								**/
	FIN (wlan_reset_sv (void));

	/* Reset only those flags that are updated during DCF operation because	*/
	/* roaming is supported only in DCF mode. The only flag that is left	*/
	/* untouched is the receiver busy flag-- this flag should be up-to-date.*/
	wlan_flags->backoff_flag       	= OPC_FALSE;
	wlan_flags->rts_sent		   	= OPC_FALSE;
	wlan_flags->rcvd_bad_packet		= OPC_FALSE;
	wlan_flags->bad_packet_dropped 	= OPC_FALSE;
	wlan_flags->transmitter_busy	= OPC_FALSE;
	wlan_flags->wait_eifs_dur		= OPC_FALSE;
	wlan_flags->immediate_xmt		= OPC_FALSE;
	wlan_flags->forced_bk_end  	    = OPC_FALSE;
	wlan_flags->cw_required			= OPC_FALSE;
	wlan_flags->perform_cw			= OPC_FALSE;
	wlan_flags->nav_updated			= OPC_FALSE;
	wlan_flags->collision			= OPC_FALSE;
	wlan_flags->duration_zero		= OPC_FALSE;
	wlan_flags->ignore_busy			= OPC_FALSE;
	wlan_flags->more_data			= OPC_FALSE;
	wlan_flags->more_frag			= OPC_FALSE;
	wlan_flags->rcvd_bad_cts		= OPC_FALSE;

	/* Initialize retry and back-off slot counts.							*/
	short_retry_count = 0;
	long_retry_count  = 0;
	backoff_slots = BACKOFF_SLOTS_UNSET;
	
	/* Initialize NAV duration.												*/
	nav_duration = op_sim_time ();
	
	/* Initialize receiver idle timer. 										*/
	rcv_idle_time = op_sim_time ();

	/* Initializing frame response to send to none.							*/
	fresp_to_send = WlanC_None;
	
	/* Initializing expected frame type to none.							*/
	expected_frame_type = WlanC_None;
	 
	FOUT;
	}

static void 
wlan_find_new_ap_virtual (void)
	{
	WlanT_AP_Position_Info	*ap_info_ptr;
	double					distance, ap_rx_pow;
	double					lat, lon, alt, x_pos, y_pos, z_pos;

	/** Find the first AP that is closer than the acceptable threshold for	**/
	/** a new connection.													**/
	FIN (wlan_find_new_ap_virtual (void));

	/* Get current position for comparison with previously cached position. */
	op_ima_obj_pos_get (my_node_objid, &lat, &lon, &alt, &x_pos, &y_pos, &z_pos);

	/* Check whether our position has changed. 								*/
	if ((roam_state_ptr->lat != lat) || (roam_state_ptr->lon != lon) || (roam_state_ptr->alt != alt))
		{
		/* Position has been changed. Cache the new position. 				*/
		roam_state_ptr->lat = lat;
		roam_state_ptr->lon = lon;
		roam_state_ptr->alt = alt;
		}

	/* Loop over all the APs in the network.								*/
	for (ap_info_ptr = global_ap_pos_info_head; ap_info_ptr != OPC_NIL; ap_info_ptr = ap_info_ptr->next_ptr)
		{
		/* Consider only those APs that are on the same channel as our		*/
		/* current channel, which we are scanning.							*/
		if (channel_num != ap_info_ptr->ap_channel_num)
			continue;

		/* Calculate the distance between this STA and an AP from the list. */
		distance = prg_geo_lat_long_distance_get (lat, lon, alt, ap_info_ptr->lat, ap_info_ptr->lon, ap_info_ptr->alt);
		
		/* Compute expected signal strength. 								*/
		ap_rx_pow = wlan_ap_signal_strength_calc (distance, ap_info_ptr, channel_num, phy_type);

		/* Pick the first AP that passes the new connection threshold		*/
		/* check. Note that the beacon-based scanning algorithm picks the	*/
		/* first reliable connection. However, the APs picked may not match	*/
		/* in the different methods.                             			*/
		if (ap_rx_pow > WLANC_ROAM_NEW_CONN_VIRTUAL_THRESH (rx_power_threshold))
			{
			/* STA has found a connection-- move it out of scan mode.		*/
			roam_state_ptr->scan_mode = OPC_FALSE;

			/* Update state variables that permit the STA to connect to the	*/
			/* new AP.														*/
			eval_bss_id = ap_info_ptr->ap_bss_id;
			wlan_set_transceiver_channel (channel_num, phy_type, txch_objid, rxch_objid);
			conn_ap_pos_info_ptr = ap_info_ptr;
			
			break;
			}
		}

	FOUT;
	}

/** Callback Functions.			**/	

static int
wlan_hld_list_elem_add_comp (const void* list_elem_ptr1,  const void* list_elem_ptr2)
	{
	WlanT_Hld_List_Elem* 	hld_ptr1;
	WlanT_Hld_List_Elem* 	hld_ptr2;
	
	/* This procedure is used in list processing to sort lists and find members of lists	*/
	/* containing higher layer data packets according to their MAC address destinations.	*/
	/* It returns 1 if hld_ptr1 is at a lower address than hld_ptr2 (closer to list head).	*/
	/* It returns -1 if hld_ptr1 is at a higher address than hld_ptr2 (closer to list tail).*/
	/* It returns 0 if the addresses associated with the two elements are equal.			*/
	FIN (wlan_hld_list_elem_add_comp (list_elem_ptr1, list_elem_ptr2));

	hld_ptr1 = (WlanT_Hld_List_Elem *) list_elem_ptr1;
	hld_ptr2 = (WlanT_Hld_List_Elem *) list_elem_ptr2;
	
	if (hld_ptr1->destination_address > hld_ptr2->destination_address) {FRET (-1);}
	if (hld_ptr1->destination_address < hld_ptr2->destination_address) {FRET (1);}
	else {FRET (0);}
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
	void wlan_mac (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Obtype _op_wlan_mac_init (int * init_block_ptr);
	void _op_wlan_mac_diag (OP_SIM_CONTEXT_ARG_OPT);
	void _op_wlan_mac_terminate (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Address _op_wlan_mac_alloc (VosT_Obtype, int);
	void _op_wlan_mac_svar (void *, const char *, void **);


#if defined (__cplusplus)
} /* end of 'extern "C"' */
#endif




/* Process model interrupt handling procedure */


void
wlan_mac (OP_SIM_CONTEXT_ARG_OPT)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	FIN_MT (wlan_mac ());

		{
		/* Temporary Variables */
		/* variables used for registering and discovering process models */
		OmsT_Pr_Handle			process_record_handle;
		List*					proc_record_handle_list_ptr;
		int						record_handle_list_size;
		int						ap_count;
		OpT_Int64				sta_addr;
		int						statype ;
		Objid					mac_objid;
		Objid					mac_if_module_objid;
		Objid					parent_subnet_objid;
		char					name_str [128];
		Objid					params_attr_objid;
		Objid					wlan_params_comp_attr_objid;
		int						i_cnt, j_cnt, k_cnt;
		int						addr_index;
		Prohandle				own_prohandle;
		WlanT_Peer_Info*		peer_info_ptr;
		void*					dummy_ptr;
		double					timer_duration;
		char					msg1 [256];
		WlanT_Phy_Char_Code		sta_phy_char_flag, ap_phy_char_flag;
		Boolean					bad_packet_rcvd = OPC_FALSE;
		Boolean					bad_cts_to_self_rcvd;
		Boolean					pre_rx_status;
		double					pcf_active;
		OpT_Int64				address;
		int						pcf_enabled_stations;
		Boolean					pcf_enabled_on_AP;
		double					tx_power;
		double					x_pos, y_pos, z_pos;
		int						integer_mac_address = -1;
		/* End of Temporary Variables */


		FSM_ENTER ("wlan_mac")

		FSM_BLOCK_SWITCH
			{
			/*---------------------------------------------------------*/
			/** state (INIT) enter executives **/
			FSM_STATE_ENTER_UNFORCED_NOLABEL (0, "INIT", "wlan_mac [INIT enter execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [INIT enter execs]", state0_enter_exec)
				{
				/* Initialization of the process model.				*/  
				/* All the attributes are loaded in this routine	*/
				wlan_mac_sv_init (); 
				
				/* Schedule a self interrupt to wait for mac interface 	*/
				/* to move to next state after registering				*/
				op_intrpt_schedule_self (op_sim_time (), 0);
				}
				FSM_PROFILE_SECTION_OUT (state0_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (1,"wlan_mac")


			/** state (INIT) exit executives **/
			FSM_STATE_EXIT_UNFORCED (0, "INIT", "wlan_mac [INIT exit execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [INIT exit execs]", state0_exit_exec)
				{
				/* Obtain the process's process handle.					*/
				own_prohandle = op_pro_self ();
				
				/* Obtain the values assigned to the various attributes.*/
				op_ima_obj_attr_get (my_objid, "Wireless LAN Parameters", &wlan_params_comp_attr_objid);
				params_attr_objid = op_topo_child (wlan_params_comp_attr_objid, OPC_OBJTYPE_GENERIC, 0);
				
				/* Obtain the name of the process.						*/
				op_ima_obj_attr_get (my_objid, "process model", name_str);
				
				/* Determine the assigned MAC address which will be		*/
				/* used for address resolution. Note this is not the	*/
				/* final MAC address as there may be static assignments	*/
				/* in the network.										*/
				op_ima_obj_attr_get (my_objid, "Address", &integer_mac_address);
				
				/* Perform auto-addressing for the MAC address. Apart	*/
				/* from dynamically addressing, if auto-assigned, the	*/
				/* address resolution function also detects duplicate	*/
				/* static assignments. The function also initializes 	*/
				/* every MAC address as a valid destination.			*/
				oms_aa_address_resolve (oms_aa_handle, my_objid, &integer_mac_address);
				my_address = integer_mac_address;
				
				/* Register Wlan MAC process in the model wide registry	*/
				own_process_record_handle = (OmsT_Pr_Handle) oms_pr_process_register (my_node_objid, my_objid, own_prohandle, name_str);
				
				/* Initialize the channels of our transmitter and		*/
				/* receiver.											*/
				wlan_transceiver_channel_init ();
				
				/* Initialize a temp variable for pcf_usage.			*/
				if (pcf_flag == OPC_BOOLINT_ENABLED) 
					pcf_active = 1.0;
				else 
					pcf_active = 0.0;
				
				/* If this station is an access point then it has to be registered as an Access Point.	*/
				/* This is because the network will be treated as Infrastructure network once AP is		*/
				/* detected.																			*/
				if (ap_flag == OPC_BOOLINT_ENABLED)
					{
					/* Allocation of record that stores position information for the AP-- STAs obtain a */
					/* pointer to this record to calculate their distance to the AP for "virtual roam". */
					conn_ap_pos_info_ptr = (WlanT_AP_Position_Info*) op_prg_mem_alloc (sizeof (WlanT_AP_Position_Info));
				
					/* Access and store node position and transmit power. */
					op_ima_obj_pos_get (my_node_objid, &(conn_ap_pos_info_ptr->lat), &(conn_ap_pos_info_ptr->lon),
						&(conn_ap_pos_info_ptr->alt), &x_pos, &y_pos, &z_pos);
					op_ima_obj_attr_get (params_attr_objid, "Transmit Power", &tx_power);
					conn_ap_pos_info_ptr->tx_power = tx_power;
				
					/* Save the BSS ID & channel number in this record-- used when the STA switches to another AP */
					conn_ap_pos_info_ptr->ap_bss_id = bss_id;
					conn_ap_pos_info_ptr->ap_channel_num = channel_num;
				
					oms_pr_attr_set (own_process_record_handle,
						"protocol",				OMSC_PR_STRING,			"mac",
						"mac_type",				OMSC_PR_STRING,			"wireless_lan",
						"subprotocol", 			OMSC_PR_INT32,			WlanC_AP,
						"domain_id",			OMSC_PR_NUMBER,			(double) bss_id,
						"PHY_type",				OMSC_PR_NUMBER,			(double) phy_type,
						"QoS support",			OMSC_PR_INT32,			(int) OPC_FALSE,
						"subnetid",				OMSC_PR_OBJID,		    my_subnet_objid,
						"ratx_objid",			OMSC_PR_OBJID,			tx_objid,
						"rarx_objid",			OMSC_PR_OBJID,			rx_objid,
						"address",			    OMSC_PR_INT64,			my_address,
						"link_speed",			OMSC_PR_NUMBER,			data_tx_rate,
						"auto address handle",  OMSC_PR_POINTER,  	    oms_aa_handle,
						"position record",  	OMSC_PR_POINTER,  	    conn_ap_pos_info_ptr,
						"PCF active",			OMSC_PR_NUMBER,			pcf_active,
						OPC_NIL);                                       
					}
				else
					{
				   	oms_pr_attr_set (own_process_record_handle,
						"protocol",				OMSC_PR_STRING,			"mac",
						"mac_type",				OMSC_PR_STRING,			"wireless_lan",
						"subprotocol", 			OMSC_PR_INT32,			WlanC_nAP_STA,
						"domain_id",			OMSC_PR_NUMBER,		    (double) bss_id,
						"PHY_type",				OMSC_PR_NUMBER,			(double) phy_type,
						"QoS support",			OMSC_PR_INT32,			(int) OPC_FALSE,
						"subnetid",				OMSC_PR_OBJID,			my_subnet_objid,
						"ratx_objid",			OMSC_PR_OBJID,			tx_objid,
						"rarx_objid",			OMSC_PR_OBJID,			rx_objid,
						"address",				OMSC_PR_INT64,			my_address,
						"link_speed",			OMSC_PR_NUMBER,			data_tx_rate,
						"auto address handle",	OMSC_PR_POINTER,		oms_aa_handle,
						"PCF active",			OMSC_PR_NUMBER,			pcf_active,
						OPC_NIL);                                       
					}
				
				/* Obtain the MAC layer information for the local MAC	*/
				/* process from the model-wide registry.				*/
				/* This is to check if the node is a gateway or not.	*/
				proc_record_handle_list_ptr = op_prg_list_create ();
				
				oms_pr_process_discover (OPC_OBJID_INVALID, proc_record_handle_list_ptr, 
					"node objid",					OMSC_PR_OBJID,			 my_node_objid,
					"protocol", 					OMSC_PR_STRING, 		 "bridge",
				 	 OPC_NIL);
				
				/* If the MAC interface process registered itself,	*/
				/* then there must be a valid match					*/
				record_handle_list_size = op_prg_list_size (proc_record_handle_list_ptr);
				
				if (record_handle_list_size != 0)
					{
					wlan_flags->bridge_flag = OPC_TRUE;
					}
				
				/* If the station is not a bridge only then check for arp	*/
				if (wlan_flags->bridge_flag == OPC_FALSE)
					{
					/* Deallocate memory used for process discovery	*/
					while (op_prg_list_size (proc_record_handle_list_ptr))
						{
						op_prg_list_remove (proc_record_handle_list_ptr, OPC_LISTPOS_HEAD);
						}
					op_prg_mem_free (proc_record_handle_list_ptr);
				
					/* Obtain the MAC layer information for the local MAC	*/
					/* process from the model-wide registry.				*/
					proc_record_handle_list_ptr = op_prg_list_create ();
					
					oms_pr_process_discover (my_objid, proc_record_handle_list_ptr, 
						"node objid",			OMSC_PR_OBJID,			my_node_objid,
						"protocol", 			OMSC_PR_STRING,			"arp", 
						OPC_NIL);
				
					/* If the MAC interface process registered itself,	*/
					/* then there must be a valid match					*/
					record_handle_list_size = op_prg_list_size (proc_record_handle_list_ptr);
					}
				
				if (record_handle_list_size != 1)
					{
					/* An error should be created if there are more	*/
					/* than one WLAN-MAC process in the local node,	*/
					/* or if no match is found.						*/
					wlan_error_print ("Either zero or several WLAN MAC interface processes found in the node.", OPC_NIL, OPC_NIL);
					}
				else
					{
					/*	Obtain a handle on the process record	*/
					process_record_handle = (OmsT_Pr_Handle) op_prg_list_access (proc_record_handle_list_ptr, OPC_LISTPOS_HEAD);
				
					/* Obtain the module objid for the Wlan MAC Interface module	*/
					oms_pr_attr_get (process_record_handle, "module objid", OMSC_PR_OBJID, &mac_if_module_objid);
				
					/* Obtain the stream numbers connected to and from the	*/
					/* Wlan MAC Interface layer process						*/
					oms_tan_neighbor_streams_find (my_objid, mac_if_module_objid, &instrm_from_mac_if, &outstrm_to_mac_if);
					}
					
				/* Deallocate memory used for process discovery	*/
				while (op_prg_list_size (proc_record_handle_list_ptr))
					{
					op_prg_list_remove (proc_record_handle_list_ptr, OPC_LISTPOS_HEAD);
					}
				
				op_prg_mem_free (proc_record_handle_list_ptr);
				
				if (debug_mode)
					{
					/* Cache the state name from which this function was	*/
					/* called.												*/
					strcpy (current_state_name, "BSS_INIT");  
					}
				}
				FSM_PROFILE_SECTION_OUT (state0_exit_exec)


			/** state (INIT) transition processing **/
			FSM_TRANSIT_FORCE (8, state8_enter_exec, ;, "default", "", "INIT", "BSS_INIT", "tr_292", "wlan_mac [INIT -> BSS_INIT : default / ]")
				/*---------------------------------------------------------*/



			/** state (IDLE) enter executives **/
			FSM_STATE_ENTER_UNFORCED (1, "IDLE", state1_enter_exec, "wlan_mac [IDLE enter execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [IDLE enter execs]", state1_enter_exec)
				{
				/** The purpose of this state is to wait until the packet has	**/
				/** arrived from the higher or lower layer.					 	**/
				/** In this state following intrpts can occur:				 	**/
				/** 1. Data arrival from application layer   				 	**/
				/** 2. Frame (DATA,ACK,RTS,CTS) rcvd from PHY layer			 	**/
				/** 3. Busy intrpt stating that frame is being rcvd			 	**/
				/** 4. Coll intrpt indicating that more than one frame is rcvd  **/
				/*																*/
				/* When Data arrives from the application layer, insert it in	*/
				/* the queue. If rcvr is not busy then set Deference to DIFS	*/
				/* and Change state to "DEFER" state. Set Backoff flag if the	*/
				/* station needs to backoff.									*/
				/*																*/
				/* Rcvd RTS,CTS,DATA,or ACK (frame rcvd intrpt):				*/
				/* If the frame is destined for this station then send			*/
				/* appropriate response and set deference to SIFS clear the		*/
				/* rcvr busy flag and clamp any data transmission.				*/
				
				if (debug_mode)
					{
					/* Determine the current state name.						*/
					strcpy (current_state_name, "IDLE");
					}
				
				/* If roaming is enabled on this node, then ensure that			*/
				/* connectivity is checked every few beacon intervals. If the	*/
				/* scan type is "virtual" (where the distance or pathloss to	*/
				/* all AP's are evaluated directly) then AP evaluation is done	*/
				/* if needed whenever any new interrupt is processed. Moreover,	*/
				/* the time at which connectivity to the AP should be next		*/
				/* evaluated is also updated each time this evaluation is		*/
				/* performed. Therefore, we need not update the next evaluation	*/
				/* time. On the other hand, if the scan type is based on 		*/
				/* actual beacon or signal strength evaluation, the next		*/
				/* evaluation time is not updated when new interrupts are		*/
				/* processed. Therefore the next evaluation time must be		*/
				/* updated. See wlan_interrupts_process() for clarification.	*/
				if (roam_state_ptr->enable_roaming) 
					{
					if (roam_state_ptr->scan_type != WlanC_Scan_Type_Distance)
						{
						ap_connectivity_check_time = current_time + beacon_int * WLANC_CONN_CHK_BEACON_MULT;
						}
					ap_connectivity_check_evhndl = op_intrpt_schedule_self (ap_connectivity_check_time, WlanC_AP_Check_Timeout);
					}
				
				/* Reset the forced backoff end flag that may have been set in	*/
				/* the BACKOFF state under rare, special cases.					*/
				wlan_flags->forced_bk_end = OPC_FALSE;
				
				/* Unlock the mutex that serializes accessing the roaming		*/
				/* related information of this MAC. 							*/
				op_prg_mt_mutex_unlock (roam_state_ptr->roam_info_mutex);
				}
				FSM_PROFILE_SECTION_OUT (state1_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (3,"wlan_mac")


			/** state (IDLE) exit executives **/
			FSM_STATE_EXIT_UNFORCED (1, "IDLE", "wlan_mac [IDLE exit execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [IDLE exit execs]", state1_exit_exec)
				{
				/* Lock the mutex that serializes accessing the roaming related		*/
				/* information of this MAC. 										*/
				op_prg_mt_mutex_lock (roam_state_ptr->roam_info_mutex, 0);
				
				/* Interrupt processing routine.									*/
				wlan_interrupts_process ();
				
				/* Schedule deference interrupt when there is a frame to transmit	*/
				/* at the stream interrupt and the receiver is not busy				*/
				if (READY_TO_TRANSMIT)
					{
					/* If the medium was idling for a period equal or longer than	*/
					/* DIFS time then we don't need to defer.						*/
					if (MEDIUM_IS_IDLE)
						{
						/* We can start the transmission immediately.				*/
						wlan_flags->immediate_xmt = OPC_TRUE;
						backoff_slots = BACKOFF_SLOTS_UNSET;
						}
					
					/* We schedule a deference only for the DCF period. 			*/
					else if (!IN_CFP)
						{
						/* We need to defer. Schedule the end of it.				*/
						wlan_schedule_deference ();
						}
					}
				
				/* If roaming is enabled, there must be a periodic self interrupt	*/
				/* scheduled for checking connectivity. Cancel this interrupt when	*/
				/* exiting this state. 												*/
				if (roam_state_ptr->enable_roaming && (intrpt_type != OPC_INTRPT_SELF || intrpt_code != WlanC_AP_Check_Timeout))
					{
					op_ev_cancel (ap_connectivity_check_evhndl);
					}
				}
				FSM_PROFILE_SECTION_OUT (state1_exit_exec)


			/** state (IDLE) transition processing **/
			FSM_PROFILE_SECTION_IN ("wlan_mac [IDLE trans conditions]", state1_trans_conds)
			FSM_INIT_COND (READY_TO_TRANSMIT && !wlan_flags->immediate_xmt)
			FSM_TEST_COND (READY_TO_TRANSMIT && wlan_flags->immediate_xmt)
			FSM_TEST_COND (AP_DISCONNECTED)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("IDLE")
			FSM_PROFILE_SECTION_OUT (state1_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 2, state2_enter_exec, ;, "READY_TO_TRANSMIT && !wlan_flags->immediate_xmt", "", "IDLE", "DEFER", "tr_233", "wlan_mac [IDLE -> DEFER : READY_TO_TRANSMIT && !wlan_flags->immediate_xmt / ]")
				FSM_CASE_TRANSIT (1, 4, state4_enter_exec, ;, "READY_TO_TRANSMIT && wlan_flags->immediate_xmt", "", "IDLE", "TRANSMIT", "tr_295", "wlan_mac [IDLE -> TRANSMIT : READY_TO_TRANSMIT && wlan_flags->immediate_xmt / ]")
				FSM_CASE_TRANSIT (2, 9, state9_enter_exec, wlan_begin_new_scan ();, "AP_DISCONNECTED", "wlan_begin_new_scan ()", "IDLE", "SCAN", "tr_302", "wlan_mac [IDLE -> SCAN : AP_DISCONNECTED / wlan_begin_new_scan ()]")
				FSM_CASE_TRANSIT (3, 1, state1_enter_exec, ;, "default", "", "IDLE", "IDLE", "tr_290", "wlan_mac [IDLE -> IDLE : default / ]")
				}
				/*---------------------------------------------------------*/



			/** state (DEFER) enter executives **/
			FSM_STATE_ENTER_UNFORCED (2, "DEFER", state2_enter_exec, "wlan_mac [DEFER enter execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [DEFER enter execs]", state2_enter_exec)
				{
				/* This state defer until the medium is available for transmission		*/
				/* interrupts that can occur in this state are:   						*/
				/* 1. Data arrival from application layer         						*/
				/* 2. Frame (DATA,ACK,RTS,CTS) rcvd from PHY layer						*/
				/* 3. Busy intrpt stating that frame is being rcvd						*/
				/* 4. Collision intrpt stating that more than one frame is rcvd    		*/
				/* 5. Deference timer has expired (self intrpt)                    		*/
				/*																		*/
				/* For Data arrival from application layer queue the packet. Set		*/
				/* Backoff flag if the station needs to backoff after deference because	*/
				/* the medium is busy. If the frame is destined for this station then	*/
				/* set frame to respond and set a deference timer to SIFS. Set			*/
				/* deference timer to SIFS and don't change states. If receiver starts	*/
				/* receiving more than one frame then flag the received frame as		*/
				/* invalid frame and set a deference to EIFS.							*/
				
				if (debug_mode)
					{
					/* Determine the current state name.								*/
					strcpy (current_state_name, "DEFER");
					}
				
				/* If in CFP, schedule a deference interrupt if not already scheduled.	*/ 
				if ((cfp_ap_medium_control == OPC_TRUE || wlan_flags->pcf_active == OPC_TRUE) &&
					op_ev_valid (deference_evh) != OPC_TRUE)
					{
					wlan_schedule_deference ();
					}
				
				
				/* Unlock the mutex that serializes accessing the roaming related		*/
				/* information of this MAC. 											*/
				op_prg_mt_mutex_unlock (roam_state_ptr->roam_info_mutex);
				}
				FSM_PROFILE_SECTION_OUT (state2_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (5,"wlan_mac")


			/** state (DEFER) exit executives **/
			FSM_STATE_EXIT_UNFORCED (2, "DEFER", "wlan_mac [DEFER exit execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [DEFER exit execs]", state2_exit_exec)
				{
				/* Lock the mutex that serializes accessing the roaming related		*/
				/* information of this MAC. 										*/
				op_prg_mt_mutex_lock (roam_state_ptr->roam_info_mutex, 0);
				
				/* Store the previous receiver status before processing the			*/
				/* interrupt, which may change the status information.				*/
				pre_rx_status = wlan_flags->receiver_busy;
				
				/* Call the interrupt processing routine for each interrupt.		*/
				wlan_interrupts_process ();
				
				if (wlan_flags->ignore_busy == OPC_FALSE)
					{
					/* If the receiver is busy while the station is deferring		*/
					/* then clear the self interrupt. As there will be a new self	*/ 
					/* interrupt generated once the receiver becomes idle again.  	*/
					if (RECEIVER_BUSY_HIGH && (op_ev_valid (deference_evh) == OPC_TRUE))
						{
						op_ev_cancel (deference_evh);
						}
				
					/* Update the value of the temporary bad packet flag, which is	*/
					/* used in the FRAME_RCVD macro below.							*/
					bad_packet_rcvd = wlan_flags->rcvd_bad_packet;
				
					/* If the receiver became idle again schedule the end of the	*/
					/* deference.													*/
					if (RECEIVER_BUSY_LOW && pre_rx_status != OPC_FALSE)
						wlan_schedule_deference ();
				
					/* While we were deferring, if we receive a frame which			*/
					/* requires a response, or we had used EIFS due to a previous 	*/
					/* error, then we need to re-schedule our end of				*/
					/* deference interrupt, since the deference time for response	*/
					/* frames is shorter. Similarly, we need to re-schedule it if	*/
					/* the received frame made us set our NAV to a higher value.	*/
					else if (FRAME_RCVD && 
							 (fresp_to_send != WlanC_None || wlan_flags->nav_updated == OPC_TRUE || 
							  ((wlan_flags->wait_eifs_dur == OPC_TRUE && IN_CFP) || wlan_flags->polled == OPC_TRUE)) && 
							 op_ev_valid (deference_evh) == OPC_TRUE)
						{
						/* Cancel the current event and schedule a new one.			*/
						op_ev_cancel (deference_evh);
						wlan_schedule_deference ();
						}
					
					/* Similarly if we received a self interrupt that indicates the	*/
					/* time to send a Beacon frame, and if we are deferring for the	*/
					/* transmission of a frame that is different then ACK frame,	*/
					/* then we need to reschedule the deference interrupt since the	*/
					/* Beacon frame will have priority over that frame, and a		*/
					/* different waiting time for deference.						*/
					else if (intrpt_type == OPC_INTRPT_SELF && intrpt_code == WlanC_Beacon_Tx_Time && fresp_to_send != WlanC_Ack &&
						     op_ev_valid (deference_evh) == OPC_TRUE)
						{
						/* Cancel the current event and schedule a new one.			*/
						op_ev_cancel (deference_evh);
						wlan_schedule_deference ();
						}
					}
				
				}
				FSM_PROFILE_SECTION_OUT (state2_exit_exec)


			/** state (DEFER) transition processing **/
			FSM_PROFILE_SECTION_IN ("wlan_mac [DEFER trans conditions]", state2_trans_conds)
			FSM_INIT_COND (DEFERENCE_OFF)
			FSM_TEST_COND (IDLE_AFTER_CFP)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("DEFER")
			FSM_PROFILE_SECTION_OUT (state2_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 3, state3_enter_exec, ;, "DEFERENCE_OFF", "", "DEFER", "BKOFF_NEEDED", "tr_223", "wlan_mac [DEFER -> BKOFF_NEEDED : DEFERENCE_OFF / ]")
				FSM_CASE_TRANSIT (1, 1, state1_enter_exec, CANCEL_DEF_EVENT;;, "IDLE_AFTER_CFP", "CANCEL_DEF_EVENT;", "DEFER", "IDLE", "tr_300", "wlan_mac [DEFER -> IDLE : IDLE_AFTER_CFP / CANCEL_DEF_EVENT;]")
				FSM_CASE_TRANSIT (2, 2, state2_enter_exec, ;, "default", "", "DEFER", "DEFER", "tr_259", "wlan_mac [DEFER -> DEFER : default / ]")
				}
				/*---------------------------------------------------------*/



			/** state (BKOFF_NEEDED) enter executives **/
			FSM_STATE_ENTER_FORCED (3, "BKOFF_NEEDED", state3_enter_exec, "wlan_mac [BKOFF_NEEDED enter execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [BKOFF_NEEDED enter execs]", state3_enter_exec)
				{
				/** In this state we determine whether a back-off is necessary for the	**/
				/** frame we are trying to transmit. It is needed when station			**/
				/** preparing to transmit frame discovers that the medium is busy or	**/
				/** when the the station infers collision. Backoff is not needed when	**/
				/** the station is responding to the frame. Following a successful		**/
				/** packet transmission, again a back-off procedure is performed for a	**/
				/** contention window period as stated in 802.11 standard.				**/
				/**																		**/
				/** If backoff needed then check whether the station completed its 		**/
				/** backoff in the last attempt. If not then resume the backoff 		**/
				/** from the same point, otherwise generate a new random number 	 	**/
				/** for the number of backoff slots. 									**/
				
				/* Checking whether backoff is needed or not.							*/ 
				if (wlan_flags->backoff_flag == OPC_TRUE || wlan_flags->perform_cw == OPC_TRUE)
					{
					if (backoff_slots == BACKOFF_SLOTS_UNSET)
						{                                                             
						/* Compute backoff interval using binary exponential process.	*/
						/* After a successful transmission we always use cw_min.		*/
						if (short_retry_count + long_retry_count == 0 || wlan_flags->perform_cw == OPC_TRUE)
							{			 
							/* If retry count is set to 0 then set the maximum backoff	*/
							/* slots to min window size.								*/	
							max_backoff = cw_min;
							}
						else
							{
							/* We are retransmitting. Increase the back-off window		*/
							/* size.													*/
							max_backoff = max_backoff * 2 + 1; 				
							}
				
						/* The number of possible slots grows exponentially until it	*/
						/* exceeds a fixed limit.										*/
						if (max_backoff > cw_max) 
							{
							max_backoff = cw_max;
							}
				 
						/* Obtain a uniformly distributed random integer between 0 and	*/
						/* the minimum contention window size. Scale the number of		*/
						/* slots according to the number of retransmissions.			*/
						backoff_slots = floor (op_dist_uniform (max_backoff + 1));
						}
				
					/* Set a timer for the end of the backoff interval.					*/
					intrpt_time = (current_time + backoff_slots * slot_time);
					
					/* Scheduling self interrupt for backoff.							*/
					if (wlan_flags->perform_cw == OPC_TRUE)
						backoff_elapsed_evh = op_intrpt_schedule_self (intrpt_time, WlanC_CW_Elapsed);
					else
						backoff_elapsed_evh = op_intrpt_schedule_self (intrpt_time, WlanC_Backoff_Elapsed);
					
					/* Reporting number of backoff slots as a statistic.				*/
					op_stat_write (backoff_slots_handle, backoff_slots);
					} 
				}
				FSM_PROFILE_SECTION_OUT (state3_enter_exec)

			/** state (BKOFF_NEEDED) exit executives **/
			FSM_STATE_EXIT_FORCED (3, "BKOFF_NEEDED", "wlan_mac [BKOFF_NEEDED exit execs]")


			/** state (BKOFF_NEEDED) transition processing **/
			FSM_PROFILE_SECTION_IN ("wlan_mac [BKOFF_NEEDED trans conditions]", state3_trans_conds)
			FSM_INIT_COND (TRANSMIT_FRAME)
			FSM_TEST_COND (PERFORM_BACKOFF)
			FSM_TEST_LOGIC ("BKOFF_NEEDED")
			FSM_PROFILE_SECTION_OUT (state3_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 4, state4_enter_exec, ;, "TRANSMIT_FRAME", "", "BKOFF_NEEDED", "TRANSMIT", "tr_242", "wlan_mac [BKOFF_NEEDED -> TRANSMIT : TRANSMIT_FRAME / ]")
				FSM_CASE_TRANSIT (1, 5, state5_enter_exec, ;, "PERFORM_BACKOFF", "", "BKOFF_NEEDED", "BACKOFF", "tr_247", "wlan_mac [BKOFF_NEEDED -> BACKOFF : PERFORM_BACKOFF / ]")
				}
				/*---------------------------------------------------------*/



			/** state (TRANSMIT) enter executives **/
			FSM_STATE_ENTER_UNFORCED (4, "TRANSMIT", state4_enter_exec, "wlan_mac [TRANSMIT enter execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [TRANSMIT enter execs]", state4_enter_exec)
				{
				/** In this state following intrpts can occur:     				**/
				/** 1. Data arrival from application layer.			        	**/
				/** 2. Frame (DATA,ACK,RTS,CTS) rcvd from PHY layer.			**/
				/** 3. Busy intrpt stating that frame is being rcvd.			**/
				/** 4. Collision intrpt means more than one frame is rcvd.  	**/
				/** 5. Transmission completed intrpt from physical layer		**/
				/** Queue the packet for Data Arrival from the higher layer,	**/
				/** and do not change state.									**/
				/** After Transmission is completed change state to FRM_END		**/
				/** No response is generated for any lower layer packet arrival	**/
				
				/* Prepare transmission frame by setting appropriate	*/
				/* fields in the control/data frame.					*/ 
				/* Skip this routine if any frame is received from the	*/
				/* higher or lower layer(s)							  	*/
				if (wlan_flags->immediate_xmt == OPC_TRUE)
					{
					/* Initialize the contention window size for the 	*/
					/* packets that are sent without backoff for the 	*/
					/* first time, if in case they are retransmitted.	*/
					max_backoff = cw_min;
					
					/* Start the transmission.							*/
					wlan_frame_transmit ();
					
					/* Reset the immediate transmission flag.			*/
					wlan_flags->immediate_xmt = OPC_FALSE;
				
					/* Also reset the EIFS flag which may have set.		*/
					wlan_flags->wait_eifs_dur = OPC_FALSE;
					}
				
				else if (wlan_flags->rcvd_bad_packet == OPC_FALSE && intrpt_type == OPC_INTRPT_SELF)  
					{
					/* If it is a PCF enabled MAC then make sure that	*/
					/* the interrupt was not PCF related. Start the		*/
					/* transmission, if the delivered self interrupt is	*/
					/* an interrupt that was just brought us into this	*/
					/* state.											*/
					if ((pcf_flag == OPC_BOOLINT_DISABLED || intrpt_code == WlanC_Deference_Off || 
						intrpt_code == WlanC_Backoff_Elapsed || intrpt_code == WlanC_CW_Elapsed) && 
						!(intrpt_code == WlanC_Beacon_Tx_Time || intrpt_code == WlanC_AP_Check_Timeout || intrpt_code == WlanC_NAV_Reset_Time))
						{
						wlan_frame_transmit ();
						
						/* Check whether the forced transmission (end	*/
						/* of backoff) flag is set.						*/
						if (wlan_flags->forced_bk_end == OPC_TRUE)
							{
							/* Reset the flag.							*/
							wlan_flags->forced_bk_end = OPC_FALSE;
							
							/* This flag indicates a rare case: at the	*/
							/* exact time when we completed our backoff	*/
							/* and started our transmission, we also	*/
							/* started receiving a packet. Hence, mark	*/
							/* the currently being received packet as a	*/
							/* bad packet.								*/
							wlan_flags->rcvd_bad_packet = OPC_TRUE;
							
							/* If we are transmitting a CTS-to-self,	*/
							/* then mark it as bad, too.				*/
							if (last_frametx_type == WlanC_Cts && expected_frame_type == WlanC_Cts)
								wlan_flags->rcvd_bad_cts = OPC_TRUE;
							}
						}
					}
				
				/* Record the state name if running in debug mode.		*/
				if (debug_mode)
					{
					strcpy (current_state_name, "TRANSMIT");
					}
				
				/* Unlock the mutex that serializes accessing the		*/
				/* roaming related information of this MAC. 			*/
				op_prg_mt_mutex_unlock (roam_state_ptr->roam_info_mutex);
				}
				FSM_PROFILE_SECTION_OUT (state4_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (9,"wlan_mac")


			/** state (TRANSMIT) exit executives **/
			FSM_STATE_EXIT_UNFORCED (4, "TRANSMIT", "wlan_mac [TRANSMIT exit execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [TRANSMIT exit execs]", state4_exit_exec)
				{
				/* Lock the mutex that serializes accessing the roaming related			*/
				/* information of this MAC. 											*/
				op_prg_mt_mutex_lock (roam_state_ptr->roam_info_mutex, 0);
				
				/* Check the interrupt type.											*/
				if (op_intrpt_type () == OPC_INTRPT_STAT)
					{
					/* If the packet is received while the the station is transmitting	*/
					/* then mark the received packet as bad if it is a WLAN frame.		*/
					intrpt_code = (WlanT_Mac_Intrpt_Code) op_intrpt_stat ();
					if (intrpt_code < TRANSMITTER_BUSY_INSTAT && op_stat_local_read (intrpt_code) > rx_power_threshold &&
						wlan_flags->rcvd_bad_packet == OPC_FALSE && 
						(wlan_flags->bad_packet_dropped == OPC_FALSE || wlan_flags->receiver_busy == OPC_FALSE) &&
						rx_state_info_ptr->busy_due_to_jammer == OPC_FALSE)
						{	
						wlan_flags->rcvd_bad_packet = OPC_TRUE;
				
						/* If we are transmitting a CTS-to-self, then mark it as bad,	*/
						/* too.															*/
						if (last_frametx_type == WlanC_Cts && expected_frame_type == WlanC_Cts)
							wlan_flags->rcvd_bad_cts = OPC_TRUE;
						}
					
					/* If we completed the transmission then reset the transmitter flag.*/
					else if (intrpt_code == TRANSMITTER_BUSY_INSTAT)
						{
						wlan_flags->transmitter_busy = OPC_FALSE;
						
						/* Also update the receiver idle time, since with the end of	*/
						/* our transmission, the medium may become idle again. If the	*/
						/* transmission requires 11g signal extension, model the 6-usec	*/
						/* no-transmission duration of signal extension	by adjusting	*/
						/* the receiver idle time accordingly. This is needed to remain	*/
						/* synchronized with other STAs in our BSS following a			*/
						/* transmission that doesn't require an acknowledgement like	*/
						/* data frames with "broadcast"	destination address (address 1).*/
						if (wlan_flags->wait_signal_ext == OPC_FALSE)
							rcv_idle_time = op_sim_time ();
						else
							{
							rcv_idle_time = op_sim_time () + WLANC_11g_SIGNAL_EXTENSION;
							wlan_flags->wait_signal_ext = OPC_FALSE;
							}
						
						/* If we transmitted a CTS-to-self that is marked bad because	*/
						/* of a colliding reception, then reset the corresponding flag,	*/
						/* if our receiver is currently not	busy. That means, the		*/
						/* reception has started and ended while we were transmitting	*/
						/* our CTS-to-self, and	therefore we can't detect that			*/
						/* collision. Hence, we have to assume that our CTS-to-self		*/
						/* transmission was	successful and to continue the frame		*/
						/* sequence	with the transmission of the data frame.			*/
						if (wlan_flags->rcvd_bad_cts == OPC_TRUE && wlan_flags->receiver_busy == OPC_FALSE)
							wlan_flags->rcvd_bad_cts = OPC_FALSE;
						}
					}
				
				else if ((op_intrpt_type () == OPC_INTRPT_STRM) && (op_intrpt_strm () != instrm_from_mac_if))
					{
					/* While transmitting, we received a packet from physical layer.	*/
					/* Mark the packet as bad.											*/
					wlan_flags->rcvd_bad_packet = OPC_TRUE;
				
					/* If we are transmitting a CTS-to-self, then mark it as bad, too.	*/
					if (last_frametx_type == WlanC_Cts && expected_frame_type == WlanC_Cts)
						wlan_flags->rcvd_bad_cts = OPC_TRUE;
					}
				
				/* Call the interrupt processing routine for each interrupt.			*/
				wlan_interrupts_process ();
				}
				FSM_PROFILE_SECTION_OUT (state4_exit_exec)


			/** state (TRANSMIT) transition processing **/
			FSM_PROFILE_SECTION_IN ("wlan_mac [TRANSMIT trans conditions]", state4_trans_conds)
			FSM_INIT_COND (TRANSMISSION_COMPLETE)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("TRANSMIT")
			FSM_PROFILE_SECTION_OUT (state4_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 6, state6_enter_exec, ;, "TRANSMISSION_COMPLETE", "", "TRANSMIT", "FRM_END", "tr_226", "wlan_mac [TRANSMIT -> FRM_END : TRANSMISSION_COMPLETE / ]")
				FSM_CASE_TRANSIT (1, 4, state4_enter_exec, ;, "default", "", "TRANSMIT", "TRANSMIT", "tr_269", "wlan_mac [TRANSMIT -> TRANSMIT : default / ]")
				}
				/*---------------------------------------------------------*/



			/** state (BACKOFF) enter executives **/
			FSM_STATE_ENTER_UNFORCED (5, "BACKOFF", state5_enter_exec, "wlan_mac [BACKOFF enter execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [BACKOFF enter execs]", state5_enter_exec)
				{
				/** Processing Random Backoff								**/
				/** In this state following intrpts can occur: 				**/
				/** 1. Data arrival from application layer   				**/
				/** 2. Frame (DATA,ACK,RTS,CTS) rcvd from PHY layer			**/
				/** 3. Busy intrpt stating that frame is being rcvd			**/
				/** 4. Coll intrpt stating that more than one frame is rcvd	**/  
				/** Queue the packet for Data Arrival from application 		**/
				/** layer and do not change the state.						**/ 
				/** If the frame is destined for this station then prepare  **/
				/** appropriate frame to respond and set deference to SIFS	**/
				/** Update NAV value (if needed) and reschedule deference	**/
				/** Change state to "DEFER"									**/
				/** If it's a broadcast frame then check whether NAV needs 	**/
				/** to be updated. Schedule self interrupt and change		**/
				/** state to Deference										**/
				/** If rcvr start receiving frame (busy stat intrpt) then 	**/
				/** set a flag indicating rcvr is busy. 					**/ 
				/** if rcvr start receiving more than one frame then flag 	**/ 
				/** the rcvd frame as invalid and set deference				**/
				/** timer to EIFS   										**/ 
				/* Change State to DEFER									**/
				
				if (debug_mode)
					{
					/* Determine the current state name.					*/
					strcpy (current_state_name, "BACKOFF");
					}
				
				/* Unlock the mutex that serializes accessing the roaming	*/
				/* related information of this MAC. 						*/
				op_prg_mt_mutex_unlock (roam_state_ptr->roam_info_mutex);
				}
				FSM_PROFILE_SECTION_OUT (state5_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (11,"wlan_mac")


			/** state (BACKOFF) exit executives **/
			FSM_STATE_EXIT_UNFORCED (5, "BACKOFF", "wlan_mac [BACKOFF exit execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [BACKOFF exit execs]", state5_exit_exec)
				{
				/* Lock the mutex that serializes accessing the roaming related		*/
				/* information of this MAC. 										*/
				op_prg_mt_mutex_lock (roam_state_ptr->roam_info_mutex, 0);
				
				/* Call the interrupt processing routine for each interrupt.		*/
				wlan_interrupts_process ();
				
				/* Set the number of slots to zero, once the backoff is completed.	*/
				if (BACKOFF_COMPLETED)
					{
					backoff_slots = BACKOFF_SLOTS_UNSET;
					}
				else if (CW_COMPLETED)
					{
					backoff_slots = BACKOFF_SLOTS_UNSET;
					
					/* Reset the contention window flags to enable future			*/
					/* transmissions.												*/
					wlan_flags->cw_required = OPC_FALSE;
					wlan_flags->perform_cw  = OPC_FALSE;
					}
				
				/* Pause the backoff procedure if our receiver just became busy or	*/
				/* if we received a self interrupt indicating the time to send a	*/
				/* Beacon frame for a new contention free period.					*/
				if (RECEIVER_BUSY_HIGH || (!wlan_flags->receiver_busy && intrpt_type == OPC_INTRPT_SELF && intrpt_code == WlanC_Beacon_Tx_Time)) 
					{
					/* Computing remaining backoff slots for next iteration.		*/
					backoff_slots =  ceil ((intrpt_time - current_time - PRECISION_RECOVERY) / slot_time);
					
					/* Don't cancel the end-of-backoff interrupt if we have already	*/
					/* completed all the slots of the back-off.						*/
					if (op_ev_valid (backoff_elapsed_evh) == OPC_TRUE && (backoff_slots > 0.0 || !RECEIVER_BUSY_HIGH))
						{
						/* Clear the self interrupt as station needs to defer.		*/
						op_ev_cancel (backoff_elapsed_evh);
						
						/* Disable perform cw flag because the station will not		*/
						/* backoff using contention window.							*/
						wlan_flags->perform_cw  = OPC_FALSE;
						}
				
					/* If the remaining backoff slots were computed as "0" slot		*/
					/* then we are experiencing a special case: we started 			*/
					/* receiving a transmission at the exact same time when we will	*/
					/* complete our backoff and start our own transmission. In such	*/
					/* cases we ignore the reception and continue with our planned	*/
					/* transmission for the accuracy of the simulation model,		*/
					/* because these two events are happening at the exact same		*/
					/* time and	their execution order should not change the overall	*/
					/* behavior of the MAC. Similarly, this will take us to IDLE	*/
					/* state in the next interrupt, if the reception started at the	*/
					/* exact time when we would complete our CW period and move		*/
					/* back to IDLE because of empty high layer buffer.				*/
					if (backoff_slots == 0.0 && intrpt_type == OPC_INTRPT_STAT)
						wlan_flags->forced_bk_end = OPC_TRUE;
					}
				}
				FSM_PROFILE_SECTION_OUT (state5_exit_exec)


			/** state (BACKOFF) transition processing **/
			FSM_PROFILE_SECTION_IN ("wlan_mac [BACKOFF trans conditions]", state5_trans_conds)
			FSM_INIT_COND (PERFORM_TRANSMIT)
			FSM_TEST_COND (BACK_TO_DEFER)
			FSM_TEST_COND (BACK_TO_IDLE)
			FSM_TEST_COND (SCAN_AFTER_CW)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("BACKOFF")
			FSM_PROFILE_SECTION_OUT (state5_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 4, state4_enter_exec, ;, "PERFORM_TRANSMIT", "", "BACKOFF", "TRANSMIT", "tr_231", "wlan_mac [BACKOFF -> TRANSMIT : PERFORM_TRANSMIT / ]")
				FSM_CASE_TRANSIT (1, 2, state2_enter_exec, wlan_schedule_deference ();;, "BACK_TO_DEFER", "wlan_schedule_deference ();", "BACKOFF", "DEFER", "tr_257", "wlan_mac [BACKOFF -> DEFER : BACK_TO_DEFER / wlan_schedule_deference ();]")
				FSM_CASE_TRANSIT (2, 1, state1_enter_exec, ;, "BACK_TO_IDLE", "", "BACKOFF", "IDLE", "tr_297", "wlan_mac [BACKOFF -> IDLE : BACK_TO_IDLE / ]")
				FSM_CASE_TRANSIT (3, 9, state9_enter_exec, wlan_begin_new_scan ();, "SCAN_AFTER_CW", "wlan_begin_new_scan ()", "BACKOFF", "SCAN", "tr_306", "wlan_mac [BACKOFF -> SCAN : SCAN_AFTER_CW / wlan_begin_new_scan ()]")
				FSM_CASE_TRANSIT (4, 5, state5_enter_exec, ;, "default", "", "BACKOFF", "BACKOFF", "tr_263", "wlan_mac [BACKOFF -> BACKOFF : default / ]")
				}
				/*---------------------------------------------------------*/



			/** state (FRM_END) enter executives **/
			FSM_STATE_ENTER_FORCED (6, "FRM_END", state6_enter_exec, "wlan_mac [FRM_END enter execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [FRM_END enter execs]", state6_enter_exec)
				{
				/** The purpose of this state is to determine the next unforced	**/
				/** state after completing transmission.					    **/
				 
				/** 3 cases											     		**/
				/** 1. If just transmitted RTS or DATA frame then wait for 	 	**/
				/** response with expected_frame_type variable set and change   **/
				/** the states to Wait for Response otherwise just DEFER for 	**/
				/** next transmission											**/
				/** 2. If expected frame is rcvd then check to see what is the	**/
				/** next frame to transmit and set appropriate deference timer	**/
				/** 2a. If all the data fragments are transmitted then check	**/
				/** whether the queue is empty or not                           **/
				/** If not then based on threshold fragment the packet 	 		**/
				/** and based on threshold decide whether to send RTS or not   	**/
				/** If there is a data to be transmitted then wait for DIFS		**/
				/**	duration before contending for the channel					**/
				/** If nothing to transmit then go to IDLE state          		**/
				/** and wait for the packet arrival from higher or lower layer	**/
				/** 3. If expected frame is not rcvd then infer collision, 		**/
				/** set backoff flag, if retry limit is not reached       		**/
				/** retransmit the frame by contending for the channel  	 	**/
				
				/* If there is no frame expected then check to see if there		*/
				/* is any other frame to transmit.								*/
				
				if (expected_frame_type == WlanC_None) 
					{
					/* If the frame needs to be retransmitted or there is   	*/
					/* something in the fragmentation buffer to transmit or the	*/
					/* station needs to respond to a frame then schedule		*/
					/* deference.												*/
					if (FRAME_TO_TRANSMIT)
						{
						/* Schedule deference before frame transmission.		*/
						wlan_schedule_deference ();
						}
					}
				else
					{
					/* The station needs to wait for the expected frame type	*/
					/* So it will set the frame timeout interrupt which will be	*/
				    /* executed if no frame is received in the set duration.   	*/
					
					/* Compute the timer duration. It is PIFS if we are an AP	*/
					/* and currently in CFP. For all other cases, the timer		*/
					/* duration is equal to "EIFS - DIFS", which is SIFS + ACK	*/
				 	/* transmission time with PLCP header.						*/
					if (wlan_flags->pcf_active)
						timer_duration = pifs_time;
					else
						timer_duration = eifs_time - difs_time;
					
					/* Schedule the timeout interrupt.							*/
					frame_timeout_evh = op_intrpt_schedule_self (current_time + timer_duration, WlanC_Frame_Timeout);
					}
				}
				FSM_PROFILE_SECTION_OUT (state6_enter_exec)

			/** state (FRM_END) exit executives **/
			FSM_STATE_EXIT_FORCED (6, "FRM_END", "wlan_mac [FRM_END exit execs]")


			/** state (FRM_END) transition processing **/
			FSM_PROFILE_SECTION_IN ("wlan_mac [FRM_END trans conditions]", state6_trans_conds)
			FSM_INIT_COND (FRM_END_TO_IDLE)
			FSM_TEST_COND (WAIT_FOR_FRAME)
			FSM_TEST_COND (FRM_END_TO_DEFER)
			FSM_TEST_COND (FRM_END_TO_SCAN)
			FSM_TEST_LOGIC ("FRM_END")
			FSM_PROFILE_SECTION_OUT (state6_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 1, state1_enter_exec, ;, "FRM_END_TO_IDLE", "", "FRM_END", "IDLE", "tr_227", "wlan_mac [FRM_END -> IDLE : FRM_END_TO_IDLE / ]")
				FSM_CASE_TRANSIT (1, 7, state7_enter_exec, ;, "WAIT_FOR_FRAME", "", "FRM_END", "WAIT_FOR_RESPONSE", "tr_228", "wlan_mac [FRM_END -> WAIT_FOR_RESPONSE : WAIT_FOR_FRAME / ]")
				FSM_CASE_TRANSIT (2, 2, state2_enter_exec, ;, "FRM_END_TO_DEFER", "", "FRM_END", "DEFER", "tr_230", "wlan_mac [FRM_END -> DEFER : FRM_END_TO_DEFER / ]")
				FSM_CASE_TRANSIT (3, 9, state9_enter_exec, wlan_begin_new_scan ();, "FRM_END_TO_SCAN", "wlan_begin_new_scan ()", "FRM_END", "SCAN", "tr_26", "wlan_mac [FRM_END -> SCAN : FRM_END_TO_SCAN / wlan_begin_new_scan ()]")
				}
				/*---------------------------------------------------------*/



			/** state (WAIT_FOR_RESPONSE) enter executives **/
			FSM_STATE_ENTER_UNFORCED (7, "WAIT_FOR_RESPONSE", state7_enter_exec, "wlan_mac [WAIT_FOR_RESPONSE enter execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [WAIT_FOR_RESPONSE enter execs]", state7_enter_exec)
				{
				/** The purpose of this state is to wait for the response after	**/
				/** transmission. The only frames which require					**/
				/** acknowledgements are RTS and DATA frame. 					**/
				/** In this state following intrpts can occur:				   	**/
				/** 1. Data arrival from application layer   					**/
				/** 2. Frame (DATA,ACK,RTS,CTS) rcvd from PHY layer				**/
				/** 3. Frame timeout if expected frame is not rcvd 				**/
				/** 4. Busy intrpt stating that frame is being rcvd           	**/
				/** 5. Collision intrpt stating that more than one frame is rcvd**/		
				/** Queue the packet as Data Arrives from application layer		**/
				/** If Rcvd unexpected frame then collision is inferred and		**/
				/** retry count is incremented							    	**/
				/** if a collision stat interrupt from the rcvr then flag the   **/
				/** received frame as bad 										**/
				
				if (debug_mode)
					{
					/* Determine the current state name.						*/
					strcpy (current_state_name, "WAIT_FOR_RESPONSE");
					}
				
				/* Unlock the mutex that serializes accessing the roaming		*/
				/* related information of this MAC. 							*/
				op_prg_mt_mutex_unlock (roam_state_ptr->roam_info_mutex);
				}
				FSM_PROFILE_SECTION_OUT (state7_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (15,"wlan_mac")


			/** state (WAIT_FOR_RESPONSE) exit executives **/
			FSM_STATE_EXIT_UNFORCED (7, "WAIT_FOR_RESPONSE", "wlan_mac [WAIT_FOR_RESPONSE exit execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [WAIT_FOR_RESPONSE exit execs]", state7_exit_exec)
				{
				/* Lock the mutex that serializes accessing the roaming related	*/
				/* information of this MAC. 									*/
				op_prg_mt_mutex_lock (roam_state_ptr->roam_info_mutex, 0);
				
				/* First store the values of the bad packet and bad CTS-to-self	*/
				/* received flags since their values are needed for the state	*/
				/* transition and the call of the function						*/
				/* wlan_interrupts_process() below may reset these flags.		*/
				bad_packet_rcvd      = wlan_flags->rcvd_bad_packet;
				bad_cts_to_self_rcvd = wlan_flags->rcvd_bad_cts;
				
				/* Determine the interrupt type and the stream index in the		*/
				/* case of stream interrupt, since this information will be		*/
				/* in the next if statement condition.							*/
				intrpt_type = op_intrpt_type ();
				if (intrpt_type == OPC_INTRPT_STRM)
					i_strm = op_intrpt_strm ();
				
				/* Clear the frame timeout interrupt once the receiver is busy	*/
				/* or the frame is received (in case of collisions, the			*/
				/* frames whose reception has started while we were				*/
				/* transmitting are excluded in the FRAME_RCVD macro).			*/
				if (((intrpt_type == OPC_INTRPT_STAT && op_intrpt_stat () < TRANSMITTER_BUSY_INSTAT && 
					  op_stat_local_read (op_intrpt_stat ()) > rx_power_threshold && !wlan_flags->receiver_busy) ||
					 FRAME_RCVD) &&
					(op_ev_valid (frame_timeout_evh) == OPC_TRUE))
					{
					op_ev_cancel (frame_timeout_evh);
					}
				
				/* Call the interrupt processing routine for each interrupt		*/
				/* request.														*/
				wlan_interrupts_process ();
				
				/* Set the flag for state transition, if we received a bad CTS-	*/
				/* to-self at this invocation. If the flag "rcvd_bad_cts" is	*/
				/* cleared at this interrupt, then this is the case.			*/
				bad_cts_to_self_rcvd = bad_cts_to_self_rcvd ^ wlan_flags->rcvd_bad_cts;
				
				/* If expected frame is not received in the set duration or the	*/
				/* there is a collision at the receiver then set the expected	*/
				/* frame type to be none because the station needs to			*/
				/* retransmit the frame.									 	*/
				if (intrpt_type == OPC_INTRPT_SELF && intrpt_code == WlanC_Frame_Timeout)
					{
					/* Setting expected frame type to none frame.				*/
					expected_frame_type = WlanC_None;
					
					/* Set the flag that indicates collision.					*/
					wlan_flags->wait_eifs_dur = OPC_TRUE;
					
					/* If PCF is active, increment poll fail and the retry		*/
					/* counter.													*/
					if (wlan_flags->pcf_active == OPC_TRUE)
						{
						/* If last frame a data frame, Increment retry counter.	*/
						if ((last_frametx_type == WlanC_Data_Poll)	||	(last_frametx_type == WlanC_Data_A_P))
							{
							pcf_retry_count++;
							}
						
						if  (wlan_flags->active_poll == OPC_TRUE) 
							{
							poll_fail_count++;
							wlan_flags->active_poll = OPC_FALSE;
							}
						
						/* Check whether further retries are possible or the	*/
						/* data frame needs to be discarded.					*/
						wlan_pcf_frame_discard();
						}
					else
						{	
						/* Increment the appropriate retransmission count. If	*/
						/* our RTS transmission or data transmission whose size	*/
						/* is smaller than RTS/CTS threshold has failed, then	*/
						/* increment short retry count; otherwise increment the	*/
						/* long retry count.									*/
						if (last_frametx_type == WlanC_Rts || wlan_flags->frame_size_req_rts == OPC_FALSE)
							short_retry_count++;
						else
							long_retry_count++;
							
						/* Also reset the rts_sent flag since we need to		*/
						/* recontend for the medium.							*/
						wlan_flags->rts_sent = OPC_FALSE;
						
						/* Reset the NAV duration so that the retransmission is	*/
						/* not unnecessarily delayed.							*/
						nav_duration = current_time;
						
						/* Check whether further retries are possible or the	*/
						/* data frame needs to be discarded.					*/
						wlan_frame_discard ();
						}
					}
				
				}
				FSM_PROFILE_SECTION_OUT (state7_exit_exec)


			/** state (WAIT_FOR_RESPONSE) transition processing **/
			FSM_PROFILE_SECTION_IN ("wlan_mac [WAIT_FOR_RESPONSE trans conditions]", state7_trans_conds)
			FSM_INIT_COND (FRAME_TIMEOUT || FRAME_RCVD)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("WAIT_FOR_RESPONSE")
			FSM_PROFILE_SECTION_OUT (state7_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 6, state6_enter_exec, ;, "FRAME_TIMEOUT || FRAME_RCVD", "", "WAIT_FOR_RESPONSE", "FRM_END", "tr_249", "wlan_mac [WAIT_FOR_RESPONSE -> FRM_END : FRAME_TIMEOUT || FRAME_RCVD / ]")
				FSM_CASE_TRANSIT (1, 7, state7_enter_exec, ;, "default", "", "WAIT_FOR_RESPONSE", "WAIT_FOR_RESPONSE", "tr_265", "wlan_mac [WAIT_FOR_RESPONSE -> WAIT_FOR_RESPONSE : default / ]")
				}
				/*---------------------------------------------------------*/



			/** state (BSS_INIT) enter executives **/
			FSM_STATE_ENTER_UNFORCED (8, "BSS_INIT", state8_enter_exec, "wlan_mac [BSS_INIT enter execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [BSS_INIT enter execs]", state8_enter_exec)
				{
				/* Schedule a self interrupt to wait for mac interface 	*/
				/* to move to next state after registering				*/
				op_intrpt_schedule_self (op_sim_time (), 0);
				}
				FSM_PROFILE_SECTION_OUT (state8_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (17,"wlan_mac")


			/** state (BSS_INIT) exit executives **/
			FSM_STATE_EXIT_UNFORCED (8, "BSS_INIT", "wlan_mac [BSS_INIT exit execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [BSS_INIT exit execs]", state8_exit_exec)
				{
				/* Obtain the values assigned to the various attributes				*/
				op_ima_obj_attr_get (my_objid, "Wireless LAN Parameters", &wlan_params_comp_attr_objid);
				params_attr_objid = op_topo_child (wlan_params_comp_attr_objid, OPC_OBJTYPE_GENERIC, 0);
				
				/* Determining the final MAC address after address resolution.		*/
				op_ima_obj_attr_get (my_objid, "Address", &integer_mac_address);
				my_address = integer_mac_address;
				
				/* Update our own process registery record with the final address	*/
				/* information.														*/
				oms_pr_attr_set (own_process_record_handle, "address", OMSC_PR_INT64, my_address, OPC_NIL);                                       
				
				/* Destroy the global list of BSS IDs that was used while assigning	*/
				/* channels to BSSs.												*/
				wlan_bss_id_list_manage (bss_id, phy_type, "destroy_list");
				
				/* Since the frequency band of the channels of all radio nodes are	*/
				/* determined, refresh our potential receiver group to exclude the	*/
				/* receiver channels that have non-overlapping bands with ours,		*/
				/* which we can't reach and interfere, to improve simulation		*/
				/* performance.														*/
				wlan_rxgroup_reduce (txch_objid, phy_char_flag, roam_state_ptr);
				
				/* Create the pool of station addresses of the same BSS using BSS	*/
				/* ID information.													*/
				wlan_sta_addr_register (bss_id, my_address, ap_flag, my_objid, phy_char_flag);
				
				/* Get a handle to the information record of our own BSS, which		*/
				/* also contains the STA list of our BSS.							*/
				my_bss_info_ptr = wlan_bss_info_get (bss_id);
				
				/* If we are operating in a bridge/switch node, indicate that our	*/
				/* address is not a valid destination address, since the station	*/
				/* models may choose their destination addresses randomly from the	*/
				/* address pool.													*/
				if (wlan_flags->bridge_flag)
					oms_aa_dest_status_set (oms_aa_handle, (int) my_address, OmsC_Aa_Invalid_Destination);
				
				/* Search for the other WLAN MACs that are in the same LAN with us.	*/
				proc_record_handle_list_ptr = op_prg_list_create ();
				
				/* If the subnet is BSS based then do the process discovery using	*/
				/* BSS ID (domain_id), otherwise do it using subnet ID.				*/
				if (bss_id_type == WlanC_Bss_Divided_Subnet)
					{
				    oms_pr_process_discover (OPC_OBJID_INVALID, proc_record_handle_list_ptr, 
					  "domain_id",			OMSC_PR_NUMBER,  		 (double) bss_id,
					  "mac_type",			OMSC_PR_STRING,	 		 "wireless_lan",
					  "protocol",			OMSC_PR_STRING,			 "mac",
					   OPC_NIL);
				    }
				else
					{
					oms_pr_process_discover (OPC_OBJID_INVALID, proc_record_handle_list_ptr, 
					  "subnetid",			OMSC_PR_OBJID,			 my_subnet_objid,
					  "mac_type",			OMSC_PR_STRING,			"wireless_lan",
					  "protocol",			OMSC_PR_STRING,			"mac",
				 	   OPC_NIL);
					}
				
				/* If the MAC interface process registered itself,	*/
				/* then there must be a valid match					*/
				record_handle_list_size = op_prg_list_size (proc_record_handle_list_ptr);
				
				/* Initialize the address list index to zero.	*/
				addr_index = 0; 
				
				/* Variable to counting number of access point in the network.	*/
				ap_count = 0;
				
				/* Initialize the variable which holds the count of PCF users in network */
				poll_list_size =0;
				
				/* Keeps track of the number of stations which are PCF enabled 	*/
				/* If this value is greater than 0, only then a Beacon frame is */
				/* transmitted at the targeted time.							*/
				pcf_enabled_stations = 0;
				
				/* Preserves the PCF status on an AP node */
				pcf_enabled_on_AP = OPC_FALSE;
				
				/* Create the binary Hash table that will be used to access the		*/
				/* information record of other STAs in our BSS. Set the table size	*/
				/* to approximately twice of the number of STAs in the BSS.			*/
				if (record_handle_list_size <= 2)
					peer_info_hash_tbl = prg_bin_hash_table_create (2, sizeof (OpT_Int64));
				else
					peer_info_hash_tbl = prg_bin_hash_table_create ((int) floor (log (record_handle_list_size * 1.5) / log (2)) + 1, sizeof (OpT_Int64));
				
				/* Traversing the process record handle list to determine if there	*/
				/* is any access point in the subnet.								*/
				for (i_cnt = 0; i_cnt < record_handle_list_size; i_cnt++ )
					{
					/*	Obtain a handle on the process record.						*/
					process_record_handle = (OmsT_Pr_Handle) op_prg_list_access (proc_record_handle_list_ptr, i_cnt);
				
					/* Get the station type.										*/
					oms_pr_attr_get (process_record_handle, "subprotocol",  OMSC_PR_INT32, &statype);
				
					/* Obtain the MAC address of the STA. Read the value of the		*/
					/* corresponding attribute to make sure that we get the final	*/
					/* value.														*/
					oms_pr_attr_get (process_record_handle, "module objid", OMSC_PR_OBJID, &mac_objid);
					op_ima_obj_attr_get (mac_objid, "Address", &integer_mac_address);
					sta_addr = integer_mac_address;
				
					/* If the station is an Access Point then its station id will be a BSS id for all the station in that subnet.	*/
					if (statype == WlanC_AP || statype == WlanC_QAP)
						{
						/* If access point found then it means that it is a			*/
						/* Infrastructured BSS.										*/
						bss_flag = OPC_TRUE;
					
						/* Record the station address of the access point.			*/
						ap_mac_address = sta_addr;
				
						/* According to IEEE 802.11 there cannot be more than one access point in	*/
						/* the same BSS.															*/
				   		ap_count = ap_count + 1;
						if (ap_count == 2)
							{
							sprintf(msg1,"More than one Access Point found within the same BSS (BSS ID = %d)" , bss_id);
							wlan_error_print (msg1,"or in the same OPNET subnet.","Check the configuration.");
							}		 
				
						/* Save position information of the connected AP for "virtual" roaming.	*/ 
						if (ap_flag == OPC_BOOLINT_DISABLED)
							oms_pr_attr_get (process_record_handle, "position record", OMSC_PR_POINTER, &conn_ap_pos_info_ptr);	 
						}
					    
					/* Checking the physical characteristic configuration for the subnet.		*/
					/* Obtain the values assigned to the various attributes						*/
					op_ima_obj_attr_get (mac_objid, "Wireless LAN Parameters", &wlan_params_comp_attr_objid);
					params_attr_objid = op_topo_child (wlan_params_comp_attr_objid, OPC_OBJTYPE_GENERIC, 0);
				
					/* Load the appropriate physical layer characteristics.						*/	
					op_ima_obj_attr_get (params_attr_objid, "Physical Characteristics", &sta_phy_char_flag);
				
					/* Store the physical layer technology used by the AP, which we may need	*/
					/* below.																	*/
					if (statype == WlanC_QAP || statype == WlanC_AP)
						ap_phy_char_flag = sta_phy_char_flag;
					
					/* Mismatching physical layer technologies within the same BSS are not		*/
					/* allowed.																	*/
					if (sta_phy_char_flag != phy_char_flag)
						{
						/* With one exception: 11g and 11/11b-DSSS STAs can coexist in the same	*/
						/* BSS.																	*/
						
						/* If we are an 11g STA, then set the non_erp_present flag if there are	*/
						/* some non-ERP (11/11b) STAs in our (I)BSS.							*/
						if (phy_char_flag == WlanC_ERP_OFDM_11g && sta_phy_char_flag == WlanC_Direct_Sequence)
							wlan_flags->non_erp_present = OPC_TRUE;
				
						/* Terminate the simulation unless it is the exception case.			*/
						else if (phy_char_flag != WlanC_Direct_Sequence || sta_phy_char_flag != WlanC_ERP_OFDM_11g)
							{
							wlan_error_print ("Physical Characteristic configuration mismatch in WLAN BSS. All stations in the same BSS (in",
								              "the same subnet if BSS IDs are auto-assigned) should have the same physical characteristics",
										      "(exception: Direct Sequence and ERP stations can coexist). Check the configuration.");
							}
						}	
					
					/* If we belong to an IBSS or we are an AP, create an information record	*/
					/* for each STA in our BSS.													*/
					if (ap_flag == OPC_BOOLINT_ENABLED || bss_flag == OPC_FALSE)
						{
						peer_info_ptr = (WlanT_Peer_Info *) op_prg_mem_alloc (sizeof (WlanT_Peer_Info));
					
						/* Initialize the last received frame seqeunce ID information, which	*/
						/* will be used for duplicate detection.								*/
						peer_info_ptr->seq_cntl = 0xFFFF;
				
						/* Record whether the peer STA supports Extended Rate PHY (11g).		*/
						peer_info_ptr->is_erp = (sta_phy_char_flag == WlanC_ERP_OFDM_11g) ? OPC_TRUE : OPC_FALSE;
						
						/* Insert the record into our local binary Hash table for quick access.	*/
						prg_bin_hash_table_item_insert (peer_info_hash_tbl, (void *) &sta_addr, peer_info_ptr, &dummy_ptr);
						}
				
					/* Check if the PCF mode has been enabled in this station.					*/
					oms_pr_attr_get (process_record_handle, "PCF active", OMSC_PR_NUMBER, &pcf_active);
					
					/* If this station has its PCF functionality enabled, increment the PCF		*/
					/* user count.																*/
					if (pcf_active != 0.0) 
						{
						poll_list_size++;
				
						/* Preserve the PCF status if it is an AP. It is required later, when    */
						/* we check for PCF functionality enabled on other nodes in the network. */
						if (statype == WlanC_AP)
							{
							pcf_enabled_on_AP = OPC_TRUE;
							}
						}	
				    }
				
				/* Do we belong to an IBSS or infrastructure BSS?								*/
				if (bss_flag == OPC_TRUE)
					{
					/* Write our BSS ID into the AP connectivity statistic since we belong to	*/
					/* an infrastructure BSS.													*/
					op_stat_write (ap_conn_handle, bss_id);
				
					/* If we are not the AP of the BSS, then we won't keep STA records.			*/
					if (ap_flag == OPC_BOOLINT_DISABLED)
						{
						prg_bin_hash_table_destroy (peer_info_hash_tbl, op_prg_mem_free);
						peer_info_hash_tbl = OPC_NIL;
						
						/* Instead just keep an information record of our AP, which we will use	*/
						/* for duplicate frame detection while receiving frames from it.		*/
						ap_peer_info_ptr = (WlanT_Peer_Info *) op_prg_mem_alloc (sizeof (WlanT_Peer_Info));
						ap_peer_info_ptr->is_erp   = (ap_phy_char_flag == WlanC_ERP_OFDM_11g) ? OPC_TRUE : OPC_FALSE;
						ap_peer_info_ptr->seq_cntl = 0xFFFF;
						}
					}
				else
					{
					/* We belong to an independent (ad-hoc) BSS. No AP connectivity.			*/
					op_stat_write (ap_conn_handle, WLANC_AP_UNCONNECTED);
					}
				
				/* Disable roaming based on the node type and type of BSS -- roaming not		*/
				/* supported for APs or for STAs that belong to an IBSS or that are				*/
				/* PCF-pollable.																*/
				if (roam_state_ptr->enable_roaming && (pcf_flag == OPC_BOOLINT_ENABLED || ap_count == 0 || ap_flag == OPC_BOOLINT_ENABLED))
					{
					roam_state_ptr->enable_roaming = OPC_FALSE;
					
					/* Write a simulation log message to report this configuration change.	*/
					op_prg_log_entry_write (config_log_handle,
						"WARNING:\n"
						" Cannot enable roaming functionality at the Wireless LAN MAC\n"
						" layer. Roaming cannot be supported by a WLAN MAC if any of\n"
						" the conditions below is true:\n"
						" a) The MAC belongs to an independent (ad-hoc) WLAN.\n"
						" b) Access point functionality is also enabled for the MAC.\n"
						" c) PCF functionality is enabled within the initial BSS of\n"
						"    the MAC and for the MAC itself, hence, making the MAC be\n"
						"    in the PCF polling-list of the BSS's AP. Currently roaming\n"
						"    is not supported with PCF functionality.\n"
						"\n"
						" If you like the MAC layer of this node to execute roaming\n"
						" procedures and switch access points when available and\n"
						" necessary, then reconfigure the WLAN parameters on this\n"
						" node and/or on the other WLAN nodes of its LAN so that\n"
						" none of the conditions above is correct.\n");	
					}
				
				/* Even if enabled, disable the roaming if the surrounding node is fixed,	*/
				/* all the parent subnets are fixed and the beacon efficiency mode is		*/
				/* enabled, which assumes that the APs are not changing the locations. In	*/
				/* other words, there is no need to run roaming functions if the distance	*/
				/* between our node and any AP in the network will not change.				*/
				else if (roam_state_ptr->enable_roaming && op_id_to_type (my_node_objid) == OPC_OBJTYPE_NDFIX &&
						 BEACON_TX_EFFICIENCY_ENABLED)
					{
					/* Make sure all the parent subnets are also fixed.	Disable the roaming	*/
					/* and enable it back if one of the parent subnets is not fixed.		*/
					parent_subnet_objid = my_subnet_objid;
					roam_state_ptr->enable_roaming = OPC_FALSE;
					do
						{		
						if (op_id_to_type (parent_subnet_objid) != OPC_OBJTYPE_SUBNET_FIX)
							roam_state_ptr->enable_roaming = OPC_TRUE;
						
						/* Move to the parent subnet.										*/
						parent_subnet_objid = op_topo_parent (parent_subnet_objid);
						}
					while (!roam_state_ptr->enable_roaming && parent_subnet_objid != OPC_OBJID_NULL);
					}
				
				/* If we are an ERP STA (i.e. an 802.11g MAC), then we may need to change	*/
				/* some of our parameters due to other STAs in our BSS.						*/
				if (phy_type == WlanC_11g_PHY)
					{
					/* If we are operating in an ad-hoc WLAN or in a BSS with non-ERP STAs	*/
					/* then increase our slot time to 20 usec and recompute the frame		*/
					/* spacing parameters, which depend on slot time and SIFS.				*/
					if (bss_flag == OPC_FALSE || wlan_flags->non_erp_present == OPC_TRUE)
						{
						wlan_slot_time_set (20E-06);
						}
					
					/* Check the existence of non-ERP STAs.									*/
					if (wlan_flags->non_erp_present)
						{
						/* Reduce the control frame data rate to 802.11/11b mandatory data	*/
						/* rate.															*/
						control_data_rate = WLANC_11b_MIN_MANDATORY_DRATE;
						
						/* Increase the minimum contention window size unless we are in a	*/
						/* BSS with an AP and our AP supports ERP data rates.				*/
						if (ap_flag == OPC_BOOLINT_DISABLED)
							{
							if (!bss_flag || ap_peer_info_ptr->is_erp == OPC_FALSE)
								cw_min = 31;
						
							/* If our AP doesn't support 11g data rates, then, if			*/
							/* necessary, lower our data transmission rate to the highest	*/
							/* possible 11b	rate so that the AP can decode our				*/
							/* transmissions.												*/
							if (bss_flag && ap_peer_info_ptr->is_erp == OPC_FALSE)
								{
								if (data_tx_rate >= 11000000.0)
									operational_speed = 11000000.0;
								else if (data_tx_rate > 5500000.0)
									operational_speed = 5500000.0;
								}
							}
						}
					}
						
				/* Record the count of the PCF enabled stations.	*/
				/* The beacon interval will be transmitted, only	*/
				/* if there has been a PCF enabled station			*/
				pcf_enabled_stations = poll_list_size;
				
				/* If PCF functionality has been enabled on any of the nodes     */
				/* if yes, it is required to have an Access Point in the network */
				if (pcf_enabled_stations > 0) 
					{
					/* Indicates the type of network (DCF only or PCF and DCF enabled nodes 	 */
					/* If pcf_network = 1, network contains either only PCF nodes or combination */
					/* If pcf_network = 0, network contains only DCF enabled nodes 				 */
					pcf_network = 1;
				
					/* The network has PCF enabled nodes, but no AP. */
					if (ap_count == 0)
						{
						sprintf (msg1,"PCF functionality has been enabled on %d station(s)", pcf_enabled_stations);
						wlan_error_print (msg1,"An Access Point is required in the network to support PCF functionality.", 
										       "Check your network configuration");
						}
					/* PCF enabled nodes present in the network, but */
					/* AP does not support PCF. Raise an error.		 */
					else if (pcf_enabled_on_AP == OPC_FALSE)
						{
						sprintf (msg1,"PCF functionality has been enabled on %d station(s)", pcf_enabled_stations);
						wlan_error_print (msg1,"The node configured as Access Point does not support PCF functionality.", 
										       "Check your network configuration");
						}
					}
				
				/* Create polling list, only if the station is an Access Point	*/
				/* and has then PCF functionality enabled  					    */
				if ((ap_flag == OPC_BOOLINT_ENABLED) && (pcf_flag == OPC_BOOLINT_ENABLED))
					{
					/* Since the current station is a AP, exclude this	*/
					/* station from the polling list					*/		
					poll_list_size--;
				
					/* Need not allocate memory for the polling list if no nodes in */
					/* the network support PCF. Also it is not necessary to create  */
					/* the polling list for the same case.							*/
					if (poll_list_size > 0)
						{
						/* Allocate memory for the polling list based on the number of PCF users.	*/
						polling_list = (OpT_Int64 *) op_prg_mem_alloc (poll_list_size * sizeof(OpT_Int64));
				
						/* Initialize polling list entries.					*/
						j_cnt = 0;
					
						/* Loop through all the stations in the current subnet	*/
						/* which were shortlisted in the discovery process.		*/
						for (i_cnt = 0; i_cnt < record_handle_list_size; i_cnt++ )
							{
							/*	Obtain a handle to the ith station from the list of processes */
							process_record_handle = (OmsT_Pr_Handle) op_prg_list_access (proc_record_handle_list_ptr, i_cnt);
						
							/* Obtain the PCF functionality status	*/
							oms_pr_attr_get (process_record_handle, "PCF active", OMSC_PR_NUMBER, &pcf_active);
						
							/* Check if PCF functionality has been enabled on this station */
							if (pcf_active != 0.0) 
								{
								/* Obtain the address of the station. Read the address from the	*/
								/* corresponding attribute to make sure that we get the final	*/
								/* value.														*/
								oms_pr_attr_get (process_record_handle, "module objid", OMSC_PR_OBJID, &mac_objid);
								op_ima_obj_attr_get (mac_objid, "Address", &integer_mac_address);	 	 
								sta_addr = integer_mac_address;
							
								/* Check if the address of station selected from the process	*/
								/* registry is not that of the current station					*/
								if (sta_addr != my_address)
									{
									/* Store the selected station address in the polling list	*/
									polling_list[j_cnt] = sta_addr;
								
									/*Increment the polling list index							*/
									j_cnt++;				
									}
								}
							}
				
						/* Sorting has been implemented below to sort and store the	*/
						/* address of	stations in the ascending order 			*/
					
						/* Sorting needs to be done only if there are more than one	*/
						/* entry in the polling list								*/
						if (poll_list_size > 1)
							{
							/* Loop through all the elements in the polling list			*/
							for (i_cnt = 0; i_cnt < poll_list_size; i_cnt++ )
								{
								/* Store the index of the ith element 						*/
								k_cnt = i_cnt;
							
								/* Loop through all the elements from starting from i+1		*/
								for	(j_cnt = (i_cnt + 1); j_cnt < poll_list_size; j_cnt++ )
									{
									if (polling_list[j_cnt] < polling_list[k_cnt]) 
										k_cnt = j_cnt;
									}
								address= polling_list[i_cnt];
								polling_list[i_cnt] = polling_list[k_cnt];
								polling_list[k_cnt] = address;
								}
							}
						}
					}
				else 
					poll_list_size = 0;
				
				/* Printing out information to ODB.	*/
				if (wlan_trace_active == OPC_TRUE)
					{
					sprintf	(msg1, "%d stations have been polled by the AP", poll_list_size);	
					op_prg_odb_print_major (msg1, OPC_NIL);
					}
				
				/* Deallocate memory used for process discovery	*/
				while (op_prg_list_size (proc_record_handle_list_ptr))
					{
					op_prg_list_remove (proc_record_handle_list_ptr, OPC_LISTPOS_HEAD);
					}
				op_prg_mem_free (proc_record_handle_list_ptr);
				
				/* Obtain the MAC layer information for the local MAC	*/
				/* process from the model-wide registry.				*/
				/* This is to check if the node is a gateway or not.	*/
				proc_record_handle_list_ptr = op_prg_list_create ();
				
				oms_pr_process_discover (OPC_OBJID_INVALID, proc_record_handle_list_ptr, 
					"node objid",					OMSC_PR_OBJID,			 my_node_objid,
					"gateway node",					OMSC_PR_STRING,			"gateway",
				 	 OPC_NIL);
				
				/* If the MAC interface process registered itself,	*/
				/* then there must be a valid match					*/
				record_handle_list_size = op_prg_list_size (proc_record_handle_list_ptr);
				
				if (record_handle_list_size != 0)
					{
					wlan_flags->gateway_flag = OPC_TRUE;
					}
				
				/* Deallocate memory used for process discovery.	*/
				while (op_prg_list_size (proc_record_handle_list_ptr))
					{
					op_prg_list_remove (proc_record_handle_list_ptr, OPC_LISTPOS_HEAD);
					}
				op_prg_mem_free (proc_record_handle_list_ptr);
				
				/* To enable roaming, all APs must send Beacons. 	*/
				if ((ap_flag == OPC_BOOLINT_ENABLED) && (beacon_tx_count == WLANC_PERIODIC_BEACON_TX || pcf_enabled_stations > 0))
					{
					/* Schedule a self interrupt to kick off the Beacon timer.	*/
					beacon_tx_time = beacon_int;
					op_intrpt_schedule_self (beacon_tx_time, WlanC_Beacon_Tx_Time);
									
					/* Initialize the remaining beacon transmission count for	*/
					/* continuous periodic transmissions.						*/
					rem_beacon_tx = WLANC_PERIODIC_BEACON_TX;
					}
				
				/* If the beacon efficiency mode is on, we will scan the best AP based on the distance. */
				if (BEACON_TX_EFFICIENCY_ENABLED)
					{
					if (roam_state_ptr->enable_roaming == OPC_TRUE)
						{
						/* Set the scan_type to distance based since beacon efficiency is on. */
						roam_state_ptr->scan_type = WlanC_Scan_Type_Distance;
						if (op_ima_sim_attr_exists ("WLAN AP Connectivity Check Interval"))
							{
							op_ima_sim_attr_get (OPC_IMA_DOUBLE, "WLAN AP Connectivity Check Interval", &ap_connectivity_check_interval);
							}
						else
							{
							/* Use the default for now and issue a warning. */
							ap_connectivity_check_interval = WLANC_CONN_CHK_DIST_INTERVAL;
							op_sim_message ("WLAN MAC Roaming", "Cannot access sim attribute \"WLAN AP Connectivity Check Interval\".");
							}
				
						ap_connectivity_check_time = ap_connectivity_check_interval;
						}
					
					if (ap_flag == OPC_BOOLINT_ENABLED)
						{
						/* Insert the AP location information in the global list. */
						wlan_ap_position_publish (conn_ap_pos_info_ptr);
						}
					}
				
				/* Create the mutex that will be used to serialize the accessing of roaming	*/
				/* information between the receiver's power stage and MAC. Since each MAC-	*/
				/* receiver pair will have its own mutex, include the MAC's address in the	*/
				/* name of the mutex.														*/
				sprintf (name_str, "MAC "OPC_INT64_FMT" Roaming Info Mutex", my_address);
				roam_state_ptr->roam_info_mutex = op_prg_mt_mutex_create (OPC_MT_MUTEX_NO_OPTIONS, 0, name_str);
				
				/* Lock the mutex. It will be locked at the beginning of exit execs, and	*/
				/* unlocked at the end of enter execs, of each unforced state excluding the	*/
				/* initialization states.													*/
				op_prg_mt_mutex_lock (roam_state_ptr->roam_info_mutex, 0);
				}
				FSM_PROFILE_SECTION_OUT (state8_exit_exec)


			/** state (BSS_INIT) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "BSS_INIT", "IDLE", "tr_293", "wlan_mac [BSS_INIT -> IDLE : default / ]")
				/*---------------------------------------------------------*/



			/** state (SCAN) enter executives **/
			FSM_STATE_ENTER_UNFORCED (9, "SCAN", state9_enter_exec, "wlan_mac [SCAN enter execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [SCAN enter execs]", state9_enter_exec)
				{
				/* Record the state name if running in debug mode.			*/
				if (debug_mode)
					{
					strcpy (current_state_name, "SCAN");
					}
				
				/* Set the flag indicating the ongoing scanning procedure.	*/
				wlan_flags->scanning = OPC_TRUE;
					
				/* Unlock the mutex that serializes accessing the roaming	*/
				/* related information of this MAC. 						*/
				op_prg_mt_mutex_unlock (roam_state_ptr->roam_info_mutex);
				}
				FSM_PROFILE_SECTION_OUT (state9_enter_exec)

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (19,"wlan_mac")


			/** state (SCAN) exit executives **/
			FSM_STATE_EXIT_UNFORCED (9, "SCAN", "wlan_mac [SCAN exit execs]")
				FSM_PROFILE_SECTION_IN ("wlan_mac [SCAN exit execs]", state9_exit_exec)
				{
				/* Lock the mutex that serializes accessing the roaming related		*/
				/* information of this MAC. 										*/
				op_prg_mt_mutex_lock (roam_state_ptr->roam_info_mutex, 0);
				
				/* Interrupt processing routine.									*/
				wlan_interrupts_process ();
				
				/* Check whether the self interrupt set for the expiry of the scan	*/
				/* period is delivered.												*/
				if (intrpt_type == OPC_INTRPT_SELF && intrpt_code == WlanC_Scan_Timeout)
					{
					switch (roam_state_ptr->scan_type)
						{
						case WlanC_Scan_Type_Distance:
							{
							/* Attempt to find a new access point based on actual	*/
							/* distances.											*/
							wlan_find_new_ap_virtual ();
							break;
							}
						case WlanC_Scan_Type_Beacon:
							{
							/* Check whether the AP of the current channel was		*/
							/* reliable during the last evaluation (scan) period.	*/
							if (roam_state_ptr->ap_reliability == WLANC_AP_RELIABLE)
								{
								/* The STA has found a connection to a reliable AP 	*/
								/* Stop scanning to stay with this new AP.			*/
								roam_state_ptr->scan_mode = OPC_FALSE;
								}
							break;
							}
						default:
							{
							break;
							}
						}
				
					/* A new AP has been found, associate with the new AP.			*/
					if (roam_state_ptr->scan_mode == OPC_FALSE)
						{
						/* Reset state variables like nav_duration, rcv_idle_time	*/
						/* etc. 													*/
						wlan_reset_sv ();
				
						/* Set the AP Connectivity stat to the BSS ID of the new AP	*/
						/* since we	are now connected to an AP again.				*/
						op_stat_write (ap_conn_handle, eval_bss_id);
					
						/* Deregister from old AP's BSS and register with the new	*/
						/* AP's BSS, unless old and new APs are same.				*/
						if (eval_bss_id != bss_id)
							wlan_ap_switch ();
				
						/* Reset the related flag since scanning is complete.		*/
						wlan_flags->scanning = OPC_FALSE;
						
						/* Write a debug message if enabled.						*/
						if (wlan_trace_active == OPC_TRUE)
							{
							sprintf (msg1, "STA connected to AP with BSS ID %d", bss_id);
							op_prg_odb_print_major (msg1, OPC_NIL, OPC_NIL);
							}
						}
					
					/* If a connection could not be established in the current		*/
					/* channel continue scanning by switching to a new channel.		*/
					else
						{
						wlan_begin_new_scan ();
						}
					}
				}
				FSM_PROFILE_SECTION_OUT (state9_exit_exec)


			/** state (SCAN) transition processing **/
			FSM_PROFILE_SECTION_IN ("wlan_mac [SCAN trans conditions]", state9_trans_conds)
			FSM_INIT_COND (AP_CONNECTED && !DATA_FRAME_TO_TX)
			FSM_TEST_COND (AP_CONNECTED && DATA_FRAME_TO_TX)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("SCAN")
			FSM_PROFILE_SECTION_OUT (state9_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 1, state1_enter_exec, ;, "AP_CONNECTED && !DATA_FRAME_TO_TX", "", "SCAN", "IDLE", "tr_307", "wlan_mac [SCAN -> IDLE : AP_CONNECTED && !DATA_FRAME_TO_TX / ]")
				FSM_CASE_TRANSIT (1, 2, state2_enter_exec, wlan_schedule_deference ();, "AP_CONNECTED && DATA_FRAME_TO_TX", "wlan_schedule_deference ()", "SCAN", "DEFER", "tr_308", "wlan_mac [SCAN -> DEFER : AP_CONNECTED && DATA_FRAME_TO_TX / wlan_schedule_deference ()]")
				FSM_CASE_TRANSIT (2, 9, state9_enter_exec, ;, "default", "", "SCAN", "SCAN", "tr_305", "wlan_mac [SCAN -> SCAN : default / ]")
				}
				/*---------------------------------------------------------*/



			}


		FSM_EXIT (0,"wlan_mac")
		}
	}




void
_op_wlan_mac_diag (OP_SIM_CONTEXT_ARG_OPT)
	{
#if defined (OPD_ALLOW_ODB)
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = __LINE__+1;
#endif

	FIN_MT (_op_wlan_mac_diag ())

	if (1)
		{

		/* Diagnostic Block */

		BINIT
		{
		int i_cnt;
		WlanT_Hld_List_Elem* hld_ptr;
		
		/* Print address and current state of the	*/
		/* MAC.										*/
		printf ("Station MAC Address: "OPC_INT64_FMT"\n\n", my_address);
		printf ("Current state name : %s\n\n", current_state_name);
		
		/* Summarize the buffer usage.				*/
		printf ("Total buffer usage: " OPC_PACKET_SIZE_FMT "/%d bits, %d packets.\n\n", total_hlpk_size, hld_max_size, total_hlpk_num);
		
		/* Print the contents of the DCF queue.		*/
		printf ("Printing the higher layer DCF queue contents (packet ids):\n");
		for (i_cnt = 0; i_cnt < op_prg_list_size (hld_list_ptr);)
			{
			hld_ptr = (WlanT_Hld_List_Elem *) op_prg_list_access (hld_list_ptr, i_cnt);
			printf ("" OPC_PACKET_ID_FMT "\t", op_pk_id (hld_ptr->pkptr));
			if ((++i_cnt % 4) == 0)
				printf ("\n");
			}
		if ((i_cnt % 4) != 0)
			printf ("\n");
		printf ("\n*Note: This list doesn't contain the packet\n");
		printf ("that is currently being transmitted, if any.\n\n");
		
		/* Print the contents of the PCF queue if	*/
		/* we are a PCF-enabled access point.		*/
		if (active_pc)
			{
			printf ("Printing the higher layer PCF queue contents (packet ids):\n");
			for (i_cnt = 0; i_cnt < op_prg_list_size (cfpd_list_ptr);)
				{
				hld_ptr = (WlanT_Hld_List_Elem*) op_prg_list_access (cfpd_list_ptr, i_cnt);
				printf ("" OPC_PACKET_ID_FMT "\t", op_pk_id (hld_ptr->pkptr));
				if ((++i_cnt % 4) == 0)
					printf ("\n");
				}
			if ((i_cnt % 4) != 0)
				printf ("\n");
			printf ("\n*Note: This list doesn't contain the packet\n");
			printf ("that is currently being transmitted, if any.\n");
			printf ("\n\n");
			}	
		
		/* Display the current values of various	*/
		/* flags used by the MAC model.				*/
		printf ("\nPrinting the values of some MAC model flags:\n");
		printf ("--------------------------------------------\n");
		printf ("receiver_busy   : %d\n", wlan_flags->receiver_busy);
		printf ("rts_sent        : %d\n", wlan_flags->rts_sent);
		printf ("tx_beacon       : %d\n", wlan_flags->tx_beacon);
		printf ("wait_eifs_dur   : %d\n", wlan_flags->wait_eifs_dur);
		printf ("pcf_active      : %d\n", wlan_flags->pcf_active);
		printf ("polled          : %d\n", wlan_flags->polled);
		printf ("rcvd_bad_packet : %d\n", wlan_flags->rcvd_bad_packet);
		printf ("non_erp_present : %d\n", wlan_flags->non_erp_present);
		printf ("cts_to_self     : %d\n", wlan_flags->cts_to_self);
		printf ("\nPrinting the values of some MAC model state variables:\n");
		printf ("------------------------------------------------------\n");
		printf ("slot_time: %.1f usec\n", slot_time * 1000000.0);
		printf ("SIFS     : %.1f usec\n", sifs_time * 1000000.0);
		printf ("PIFS     : %.1f usec\n", pifs_time * 1000000.0);
		printf ("DIFS     : %.1f usec\n", difs_time * 1000000.0);
		printf ("EIFS     : %.1f usec\n", eifs_time * 1000000.0);
		printf ("CWmin    : %d\n\n", cw_min);
		printf ("last_frametx_type  : %d\n", last_frametx_type);
		printf ("expected_frame_type: %d\n", expected_frame_type);
		printf ("destination        : "OPC_INT64_FMT"\n", (wlan_flags->pcf_active == OPC_TRUE) ? pcf_destination_addr : dcf_destination_addr);
		printf ("rcv_idle_time      : %.8f\n", rcv_idle_time);
		printf ("nav_duration       : %.8f\n", nav_duration);
		printf ("retry_count        : %d\n", short_retry_count + long_retry_count);
		printf ("pcf_retry_count    : %d\n", pcf_retry_count);
		printf ("poll index         : %d\n", poll_index);
		}

		/* End of Diagnostic Block */

		}

	FOUT
#endif /* OPD_ALLOW_ODB */
	}




void
_op_wlan_mac_terminate (OP_SIM_CONTEXT_ARG_OPT)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = __LINE__;
#endif

	FIN_MT (_op_wlan_mac_terminate ())

	if (1)
		{

		/* Termination Block */

		BINIT
		{
		
		}

		/* End of Termination Block */

		}
	Vos_Poolmem_Dealloc (op_sv_ptr);

	FOUT
	}


/* Undefine shortcuts to state variables to avoid */
/* syntax error in direct access to fields of */
/* local variable prs_ptr in _op_wlan_mac_svar function. */
#undef intrpt_type
#undef intrpt_code
#undef my_address
#undef my_objid
#undef my_node_objid
#undef my_subnet_objid
#undef tx_objid
#undef txch_objid
#undef rx_objid
#undef rxch_objid
#undef own_process_record_handle
#undef hld_list_ptr
#undef data_tx_rate
#undef operational_speed
#undef control_data_rate
#undef rcvd_frame_drate
#undef frag_threshold
#undef packet_seq_counter
#undef packet_seq_control
#undef dcf_destination_addr
#undef dcf_orig_source_addr
#undef hl_protocol_dcf
#undef pcf_destination_addr
#undef pcf_orig_source_addr
#undef hl_protocol_pcf
#undef packet_frag_number
#undef pcf_packet_frag_number
#undef fragmentation_buffer_ptr
#undef common_rsmbuf_ptr
#undef mac_client_reassembly_buffer
#undef fresp_to_send
#undef nav_duration
#undef nav_reset_evh
#undef rts_threshold
#undef duplicate_entry
#undef expected_frame_type
#undef backoff_slots
#undef remote_sta_addr
#undef packet_load_handle
#undef intrpt_time
#undef wlan_transmit_frame_copy_ptr
#undef backoff_slots_handle
#undef instrm_from_mac_if
#undef outstrm_to_mac_if
#undef num_fragments
#undef remainder_size
#undef defragmentation_list_ptr
#undef wlan_flags
#undef oms_aa_handle
#undef current_time
#undef rcv_idle_time
#undef hld_pmh
#undef max_backoff
#undef current_state_name
#undef hl_packets_rcvd
#undef media_access_delay
#undef ete_delay_handle
#undef global_ete_delay_handle
#undef global_throughput_handle
#undef global_load_handle
#undef global_buffer_drop_handle
#undef global_retx_drop_handle
#undef global_mac_delay_handle
#undef global_retrans_handle
#undef global_network_load_handle
#undef ctrl_traffic_rcvd_handle_inbits
#undef ctrl_traffic_sent_handle_inbits
#undef ctrl_traffic_rcvd_handle
#undef ctrl_traffic_sent_handle
#undef mgmt_traffic_rcvd_handle_inbits
#undef mgmt_traffic_sent_handle_inbits
#undef mgmt_traffic_rcvd_handle
#undef mgmt_traffic_sent_handle
#undef data_traffic_rcvd_handle_inbits
#undef data_traffic_sent_handle_inbits
#undef data_traffic_rcvd_handle
#undef data_traffic_sent_handle
#undef sifs_time
#undef slot_time
#undef cw_min
#undef cw_max
#undef difs_time
#undef plcp_overhead_control
#undef plcp_overhead_data
#undef retrans_handle
#undef throughput_handle
#undef ap_conn_handle
#undef long_retry_limit
#undef short_retry_limit
#undef long_retry_count
#undef short_retry_count
#undef last_frametx_type
#undef deference_evh
#undef backoff_elapsed_evh
#undef frame_timeout_evh
#undef eifs_time
#undef i_strm
#undef wlan_trace_active
#undef pkt_in_service
#undef bits_load_handle
#undef ap_flag
#undef bss_flag
#undef ap_mac_address
#undef hld_max_size
#undef max_receive_lifetime
#undef accept_large_packets
#undef phy_char_flag
#undef phy_type
#undef total_hlpk_size
#undef total_hlpk_num
#undef buffer_drop_pkts_handle
#undef buffer_drop_bits_handle
#undef retx_drop_pkts_handle
#undef retx_drop_bits_handle
#undef drop_pkt_log_handle
#undef config_log_handle
#undef drop_pkt_entry_log_flag
#undef receive_time
#undef llc_iciptr
#undef rx_power_threshold
#undef bss_id
#undef pcf_retry_count
#undef poll_fail_count
#undef max_poll_fails
#undef cfpd_list_ptr
#undef pcf_queue_offset
#undef beacon_int
#undef beacon_tx_time
#undef pcf_frag_buffer_ptr
#undef wlan_pcf_transmit_frame_copy_ptr
#undef pcf_num_fragments
#undef pcf_remainder_size
#undef polling_list
#undef poll_list_size
#undef poll_index
#undef pifs_time
#undef cfp_end_evh
#undef pcf_pkt_in_service
#undef pcf_flag
#undef active_pc
#undef cfp_prd
#undef cfp_offset
#undef cfp_length
#undef packet_size_dcf
#undef packet_size_pcf
#undef receive_time_dcf
#undef receive_time_pcf
#undef cfp_ap_medium_control
#undef pcf_network
#undef beacon_tx_count
#undef rem_beacon_tx
#undef channel_count
#undef channel_num
#undef first_chan_min_freq
#undef channel_bandwidth
#undef channel_spacing
#undef eval_bss_id
#undef roam_state_ptr
#undef rx_state_info_ptr
#undef ap_connectivity_check_interval
#undef ap_connectivity_check_time
#undef ap_connectivity_check_evhndl
#undef conn_ap_pos_info_ptr
#undef my_bss_info_ptr
#undef ap_peer_info_ptr
#undef peer_info_hash_tbl
#undef debug_mode
#undef mapping_info_mutex
#undef data_packet_type
#undef data_packet_dest
#undef data_packet_final_dest

#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE

#define FIN_PREAMBLE_DEC
#define FIN_PREAMBLE_CODE

VosT_Obtype
_op_wlan_mac_init (int * init_block_ptr)
	{
	VosT_Obtype obtype = OPC_NIL;
	FIN_MT (_op_wlan_mac_init (init_block_ptr))

	obtype = Vos_Define_Object_Prstate ("proc state vars (wlan_mac)",
		sizeof (wlan_mac_state));
	*init_block_ptr = 0;

	FRET (obtype)
	}

VosT_Address
_op_wlan_mac_alloc (VosT_Obtype obtype, int init_block)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	wlan_mac_state * ptr;
	FIN_MT (_op_wlan_mac_alloc (obtype))

	ptr = (wlan_mac_state *)Vos_Alloc_Object (obtype);
	if (ptr != OPC_NIL)
		{
		ptr->_op_current_block = init_block;
#if defined (OPD_ALLOW_ODB)
		ptr->_op_current_state = "wlan_mac [INIT enter execs]";
#endif
		}
	FRET ((VosT_Address)ptr)
	}



void
_op_wlan_mac_svar (void * gen_ptr, const char * var_name, void ** var_p_ptr)
	{
	wlan_mac_state		*prs_ptr;

	FIN_MT (_op_wlan_mac_svar (gen_ptr, var_name, var_p_ptr))

	if (var_name == OPC_NIL)
		{
		*var_p_ptr = (void *)OPC_NIL;
		FOUT
		}
	prs_ptr = (wlan_mac_state *)gen_ptr;

	if (strcmp ("intrpt_type" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->intrpt_type);
		FOUT
		}
	if (strcmp ("intrpt_code" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->intrpt_code);
		FOUT
		}
	if (strcmp ("my_address" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_address);
		FOUT
		}
	if (strcmp ("my_objid" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_objid);
		FOUT
		}
	if (strcmp ("my_node_objid" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_node_objid);
		FOUT
		}
	if (strcmp ("my_subnet_objid" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_subnet_objid);
		FOUT
		}
	if (strcmp ("tx_objid" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->tx_objid);
		FOUT
		}
	if (strcmp ("txch_objid" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->txch_objid);
		FOUT
		}
	if (strcmp ("rx_objid" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->rx_objid);
		FOUT
		}
	if (strcmp ("rxch_objid" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->rxch_objid);
		FOUT
		}
	if (strcmp ("own_process_record_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->own_process_record_handle);
		FOUT
		}
	if (strcmp ("hld_list_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->hld_list_ptr);
		FOUT
		}
	if (strcmp ("data_tx_rate" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_tx_rate);
		FOUT
		}
	if (strcmp ("operational_speed" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->operational_speed);
		FOUT
		}
	if (strcmp ("control_data_rate" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->control_data_rate);
		FOUT
		}
	if (strcmp ("rcvd_frame_drate" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->rcvd_frame_drate);
		FOUT
		}
	if (strcmp ("frag_threshold" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->frag_threshold);
		FOUT
		}
	if (strcmp ("packet_seq_counter" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->packet_seq_counter);
		FOUT
		}
	if (strcmp ("packet_seq_control" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->packet_seq_control);
		FOUT
		}
	if (strcmp ("dcf_destination_addr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->dcf_destination_addr);
		FOUT
		}
	if (strcmp ("dcf_orig_source_addr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->dcf_orig_source_addr);
		FOUT
		}
	if (strcmp ("hl_protocol_dcf" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->hl_protocol_dcf);
		FOUT
		}
	if (strcmp ("pcf_destination_addr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pcf_destination_addr);
		FOUT
		}
	if (strcmp ("pcf_orig_source_addr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pcf_orig_source_addr);
		FOUT
		}
	if (strcmp ("hl_protocol_pcf" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->hl_protocol_pcf);
		FOUT
		}
	if (strcmp ("packet_frag_number" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->packet_frag_number);
		FOUT
		}
	if (strcmp ("pcf_packet_frag_number" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pcf_packet_frag_number);
		FOUT
		}
	if (strcmp ("fragmentation_buffer_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->fragmentation_buffer_ptr);
		FOUT
		}
	if (strcmp ("common_rsmbuf_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->common_rsmbuf_ptr);
		FOUT
		}
	if (strcmp ("mac_client_reassembly_buffer" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->mac_client_reassembly_buffer);
		FOUT
		}
	if (strcmp ("fresp_to_send" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->fresp_to_send);
		FOUT
		}
	if (strcmp ("nav_duration" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->nav_duration);
		FOUT
		}
	if (strcmp ("nav_reset_evh" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->nav_reset_evh);
		FOUT
		}
	if (strcmp ("rts_threshold" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->rts_threshold);
		FOUT
		}
	if (strcmp ("duplicate_entry" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->duplicate_entry);
		FOUT
		}
	if (strcmp ("expected_frame_type" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->expected_frame_type);
		FOUT
		}
	if (strcmp ("backoff_slots" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->backoff_slots);
		FOUT
		}
	if (strcmp ("remote_sta_addr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->remote_sta_addr);
		FOUT
		}
	if (strcmp ("packet_load_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->packet_load_handle);
		FOUT
		}
	if (strcmp ("intrpt_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->intrpt_time);
		FOUT
		}
	if (strcmp ("wlan_transmit_frame_copy_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->wlan_transmit_frame_copy_ptr);
		FOUT
		}
	if (strcmp ("backoff_slots_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->backoff_slots_handle);
		FOUT
		}
	if (strcmp ("instrm_from_mac_if" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->instrm_from_mac_if);
		FOUT
		}
	if (strcmp ("outstrm_to_mac_if" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->outstrm_to_mac_if);
		FOUT
		}
	if (strcmp ("num_fragments" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->num_fragments);
		FOUT
		}
	if (strcmp ("remainder_size" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->remainder_size);
		FOUT
		}
	if (strcmp ("defragmentation_list_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->defragmentation_list_ptr);
		FOUT
		}
	if (strcmp ("wlan_flags" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->wlan_flags);
		FOUT
		}
	if (strcmp ("oms_aa_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->oms_aa_handle);
		FOUT
		}
	if (strcmp ("current_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->current_time);
		FOUT
		}
	if (strcmp ("rcv_idle_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->rcv_idle_time);
		FOUT
		}
	if (strcmp ("hld_pmh" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->hld_pmh);
		FOUT
		}
	if (strcmp ("max_backoff" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->max_backoff);
		FOUT
		}
	if (strcmp ("current_state_name" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->current_state_name);
		FOUT
		}
	if (strcmp ("hl_packets_rcvd" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->hl_packets_rcvd);
		FOUT
		}
	if (strcmp ("media_access_delay" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->media_access_delay);
		FOUT
		}
	if (strcmp ("ete_delay_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ete_delay_handle);
		FOUT
		}
	if (strcmp ("global_ete_delay_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->global_ete_delay_handle);
		FOUT
		}
	if (strcmp ("global_throughput_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->global_throughput_handle);
		FOUT
		}
	if (strcmp ("global_load_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->global_load_handle);
		FOUT
		}
	if (strcmp ("global_buffer_drop_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->global_buffer_drop_handle);
		FOUT
		}
	if (strcmp ("global_retx_drop_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->global_retx_drop_handle);
		FOUT
		}
	if (strcmp ("global_mac_delay_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->global_mac_delay_handle);
		FOUT
		}
	if (strcmp ("global_retrans_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->global_retrans_handle);
		FOUT
		}
	if (strcmp ("global_network_load_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->global_network_load_handle);
		FOUT
		}
	if (strcmp ("ctrl_traffic_rcvd_handle_inbits" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ctrl_traffic_rcvd_handle_inbits);
		FOUT
		}
	if (strcmp ("ctrl_traffic_sent_handle_inbits" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ctrl_traffic_sent_handle_inbits);
		FOUT
		}
	if (strcmp ("ctrl_traffic_rcvd_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ctrl_traffic_rcvd_handle);
		FOUT
		}
	if (strcmp ("ctrl_traffic_sent_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ctrl_traffic_sent_handle);
		FOUT
		}
	if (strcmp ("mgmt_traffic_rcvd_handle_inbits" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->mgmt_traffic_rcvd_handle_inbits);
		FOUT
		}
	if (strcmp ("mgmt_traffic_sent_handle_inbits" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->mgmt_traffic_sent_handle_inbits);
		FOUT
		}
	if (strcmp ("mgmt_traffic_rcvd_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->mgmt_traffic_rcvd_handle);
		FOUT
		}
	if (strcmp ("mgmt_traffic_sent_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->mgmt_traffic_sent_handle);
		FOUT
		}
	if (strcmp ("data_traffic_rcvd_handle_inbits" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_traffic_rcvd_handle_inbits);
		FOUT
		}
	if (strcmp ("data_traffic_sent_handle_inbits" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_traffic_sent_handle_inbits);
		FOUT
		}
	if (strcmp ("data_traffic_rcvd_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_traffic_rcvd_handle);
		FOUT
		}
	if (strcmp ("data_traffic_sent_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_traffic_sent_handle);
		FOUT
		}
	if (strcmp ("sifs_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->sifs_time);
		FOUT
		}
	if (strcmp ("slot_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->slot_time);
		FOUT
		}
	if (strcmp ("cw_min" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->cw_min);
		FOUT
		}
	if (strcmp ("cw_max" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->cw_max);
		FOUT
		}
	if (strcmp ("difs_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->difs_time);
		FOUT
		}
	if (strcmp ("plcp_overhead_control" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->plcp_overhead_control);
		FOUT
		}
	if (strcmp ("plcp_overhead_data" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->plcp_overhead_data);
		FOUT
		}
	if (strcmp ("retrans_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->retrans_handle);
		FOUT
		}
	if (strcmp ("throughput_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->throughput_handle);
		FOUT
		}
	if (strcmp ("ap_conn_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ap_conn_handle);
		FOUT
		}
	if (strcmp ("long_retry_limit" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->long_retry_limit);
		FOUT
		}
	if (strcmp ("short_retry_limit" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->short_retry_limit);
		FOUT
		}
	if (strcmp ("long_retry_count" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->long_retry_count);
		FOUT
		}
	if (strcmp ("short_retry_count" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->short_retry_count);
		FOUT
		}
	if (strcmp ("last_frametx_type" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->last_frametx_type);
		FOUT
		}
	if (strcmp ("deference_evh" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->deference_evh);
		FOUT
		}
	if (strcmp ("backoff_elapsed_evh" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->backoff_elapsed_evh);
		FOUT
		}
	if (strcmp ("frame_timeout_evh" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->frame_timeout_evh);
		FOUT
		}
	if (strcmp ("eifs_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->eifs_time);
		FOUT
		}
	if (strcmp ("i_strm" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->i_strm);
		FOUT
		}
	if (strcmp ("wlan_trace_active" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->wlan_trace_active);
		FOUT
		}
	if (strcmp ("pkt_in_service" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pkt_in_service);
		FOUT
		}
	if (strcmp ("bits_load_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->bits_load_handle);
		FOUT
		}
	if (strcmp ("ap_flag" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ap_flag);
		FOUT
		}
	if (strcmp ("bss_flag" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->bss_flag);
		FOUT
		}
	if (strcmp ("ap_mac_address" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ap_mac_address);
		FOUT
		}
	if (strcmp ("hld_max_size" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->hld_max_size);
		FOUT
		}
	if (strcmp ("max_receive_lifetime" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->max_receive_lifetime);
		FOUT
		}
	if (strcmp ("accept_large_packets" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->accept_large_packets);
		FOUT
		}
	if (strcmp ("phy_char_flag" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->phy_char_flag);
		FOUT
		}
	if (strcmp ("phy_type" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->phy_type);
		FOUT
		}
	if (strcmp ("total_hlpk_size" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->total_hlpk_size);
		FOUT
		}
	if (strcmp ("total_hlpk_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->total_hlpk_num);
		FOUT
		}
	if (strcmp ("buffer_drop_pkts_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->buffer_drop_pkts_handle);
		FOUT
		}
	if (strcmp ("buffer_drop_bits_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->buffer_drop_bits_handle);
		FOUT
		}
	if (strcmp ("retx_drop_pkts_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->retx_drop_pkts_handle);
		FOUT
		}
	if (strcmp ("retx_drop_bits_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->retx_drop_bits_handle);
		FOUT
		}
	if (strcmp ("drop_pkt_log_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->drop_pkt_log_handle);
		FOUT
		}
	if (strcmp ("config_log_handle" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->config_log_handle);
		FOUT
		}
	if (strcmp ("drop_pkt_entry_log_flag" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->drop_pkt_entry_log_flag);
		FOUT
		}
	if (strcmp ("receive_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->receive_time);
		FOUT
		}
	if (strcmp ("llc_iciptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->llc_iciptr);
		FOUT
		}
	if (strcmp ("rx_power_threshold" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->rx_power_threshold);
		FOUT
		}
	if (strcmp ("bss_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->bss_id);
		FOUT
		}
	if (strcmp ("pcf_retry_count" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pcf_retry_count);
		FOUT
		}
	if (strcmp ("poll_fail_count" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->poll_fail_count);
		FOUT
		}
	if (strcmp ("max_poll_fails" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->max_poll_fails);
		FOUT
		}
	if (strcmp ("cfpd_list_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->cfpd_list_ptr);
		FOUT
		}
	if (strcmp ("pcf_queue_offset" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pcf_queue_offset);
		FOUT
		}
	if (strcmp ("beacon_int" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->beacon_int);
		FOUT
		}
	if (strcmp ("beacon_tx_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->beacon_tx_time);
		FOUT
		}
	if (strcmp ("pcf_frag_buffer_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pcf_frag_buffer_ptr);
		FOUT
		}
	if (strcmp ("wlan_pcf_transmit_frame_copy_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->wlan_pcf_transmit_frame_copy_ptr);
		FOUT
		}
	if (strcmp ("pcf_num_fragments" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pcf_num_fragments);
		FOUT
		}
	if (strcmp ("pcf_remainder_size" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pcf_remainder_size);
		FOUT
		}
	if (strcmp ("polling_list" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->polling_list);
		FOUT
		}
	if (strcmp ("poll_list_size" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->poll_list_size);
		FOUT
		}
	if (strcmp ("poll_index" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->poll_index);
		FOUT
		}
	if (strcmp ("pifs_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pifs_time);
		FOUT
		}
	if (strcmp ("cfp_end_evh" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->cfp_end_evh);
		FOUT
		}
	if (strcmp ("pcf_pkt_in_service" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pcf_pkt_in_service);
		FOUT
		}
	if (strcmp ("pcf_flag" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pcf_flag);
		FOUT
		}
	if (strcmp ("active_pc" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->active_pc);
		FOUT
		}
	if (strcmp ("cfp_prd" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->cfp_prd);
		FOUT
		}
	if (strcmp ("cfp_offset" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->cfp_offset);
		FOUT
		}
	if (strcmp ("cfp_length" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->cfp_length);
		FOUT
		}
	if (strcmp ("packet_size_dcf" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->packet_size_dcf);
		FOUT
		}
	if (strcmp ("packet_size_pcf" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->packet_size_pcf);
		FOUT
		}
	if (strcmp ("receive_time_dcf" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->receive_time_dcf);
		FOUT
		}
	if (strcmp ("receive_time_pcf" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->receive_time_pcf);
		FOUT
		}
	if (strcmp ("cfp_ap_medium_control" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->cfp_ap_medium_control);
		FOUT
		}
	if (strcmp ("pcf_network" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pcf_network);
		FOUT
		}
	if (strcmp ("beacon_tx_count" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->beacon_tx_count);
		FOUT
		}
	if (strcmp ("rem_beacon_tx" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->rem_beacon_tx);
		FOUT
		}
	if (strcmp ("channel_count" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->channel_count);
		FOUT
		}
	if (strcmp ("channel_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->channel_num);
		FOUT
		}
	if (strcmp ("first_chan_min_freq" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->first_chan_min_freq);
		FOUT
		}
	if (strcmp ("channel_bandwidth" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->channel_bandwidth);
		FOUT
		}
	if (strcmp ("channel_spacing" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->channel_spacing);
		FOUT
		}
	if (strcmp ("eval_bss_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->eval_bss_id);
		FOUT
		}
	if (strcmp ("roam_state_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->roam_state_ptr);
		FOUT
		}
	if (strcmp ("rx_state_info_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->rx_state_info_ptr);
		FOUT
		}
	if (strcmp ("ap_connectivity_check_interval" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ap_connectivity_check_interval);
		FOUT
		}
	if (strcmp ("ap_connectivity_check_time" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ap_connectivity_check_time);
		FOUT
		}
	if (strcmp ("ap_connectivity_check_evhndl" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ap_connectivity_check_evhndl);
		FOUT
		}
	if (strcmp ("conn_ap_pos_info_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->conn_ap_pos_info_ptr);
		FOUT
		}
	if (strcmp ("my_bss_info_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->my_bss_info_ptr);
		FOUT
		}
	if (strcmp ("ap_peer_info_ptr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->ap_peer_info_ptr);
		FOUT
		}
	if (strcmp ("peer_info_hash_tbl" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->peer_info_hash_tbl);
		FOUT
		}
	if (strcmp ("debug_mode" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->debug_mode);
		FOUT
		}
	if (strcmp ("mapping_info_mutex" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->mapping_info_mutex);
		FOUT
		}
	if (strcmp ("data_packet_type" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_packet_type);
		FOUT
		}
	if (strcmp ("data_packet_dest" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_packet_dest);
		FOUT
		}
	if (strcmp ("data_packet_final_dest" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->data_packet_final_dest);
		FOUT
		}
	*var_p_ptr = (void *)OPC_NIL;

	FOUT
	}

