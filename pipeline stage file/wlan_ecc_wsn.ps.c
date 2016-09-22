/* wlan_ecc_wsn.ps.c */
/* Modified for IEEE802.11										*/		 
/* Error correction model for radio link Transceiver Pipeline 	*/

/****************************************/
/*		Copyright (c) 1993-2008		*/
/*		by OPNET Technologies, Inc.		*/
/*		(A Delaware Corporation)		*/
/*	7255 Woodmont Av., Suite 250  		*/
/*     Bethesda, MD 20814, U.S.A.       */
/*			All Rights Reserved.		*/
/****************************************/

#include <opnet.h>
#include <wlan_support.h>	
#include <oms_tan.h>
#include <jammers.h> 
#include <string.h>  

#if defined (__cplusplus)
extern "C"
#endif
void
wlan_ecc_wsn_mt (OP_SIM_CONTEXT_ARG_OPT_COMMA Packet* pkptr)
    {
	OpT_Packet_Size			pklen;
	int						num_errs, accept, current_status;
	Objid					rx_ch_objid;
	WlanT_Phy_Char_Code		rcvd_frame_phy_char;
	WlanT_Rx_State_Info*	rx_state_ptr;
	double					ecc_thresh;
	double					pk_reception_end_with_se;

//自定义变量
	Objid			rx_node_id;
	Boolean			energy_exhaust;
	double			energy,E_current_use;
	double			E_elec;
	char			file_name[300];
	char			node_type[10]="type";
	FILE 			*in;
	int				node_address;
//自定义变量结束
	
	/** Determine acceptability of given packet at receiver. 		**/
	FIN_MT (wlan_ecc_wsn (pkptr));

//自添加代码:energy model
	
	//能量消耗计算并统计
	rx_ch_objid   = op_td_get_int (pkptr, OPC_TDA_RA_RX_CH_OBJID);
	rx_node_id = op_topo_parent (op_topo_parent (op_topo_parent (rx_ch_objid)));
	if(op_ima_obj_attr_exists(rx_node_id,"Node Type"))
	{
		op_ima_obj_attr_get(rx_node_id,"Node Type",node_type);
	}
	
	if((strcmp(node_type,"sensor")==0)&&op_ima_obj_attr_exists(rx_node_id,"Energy Exhaust")&&op_ima_obj_attr_exists(rx_node_id,"Communication Range")&&
		op_ima_obj_attr_exists(rx_node_id,"Energy")&&op_ima_obj_attr_exists(rx_node_id,"E elec")&&
		op_ima_obj_attr_exists(rx_node_id,"E amp")&&op_ima_obj_attr_exists(rx_node_id,"Invalid Time Record File"))
	{
		printf("在管道 13 阶段，是sensor节点，且节点的各 Model 属性名称正确, 开始进行能量消耗计算.\n");
		
		op_ima_obj_attr_get(rx_node_id,"Energy Exhaust", &energy_exhaust);
		if(energy_exhaust == OPC_FALSE)//能量还未尽，计算能量
		{
			op_ima_obj_attr_get(rx_node_id,"Energy",&energy);
			op_ima_obj_attr_get(rx_node_id,"E elec",&E_elec);
			pklen = op_pk_total_size_get (pkptr);
			
			E_current_use = E_elec*pklen;//按照LEACH的能量计算公式计算
			
			energy=energy-E_current_use;
			op_ima_obj_attr_set(rx_node_id,"Energy",energy);
			if(energy<=0)
			{
				op_ima_obj_attr_set(rx_node_id,"Energy Exhaust", OPC_TRUE);
				
				op_ima_obj_attr_get(rx_node_id,"Invalid Time Record File", file_name);
				op_ima_obj_attr_get(rx_node_id,"MAC Address",&node_address);
				in = fopen(file_name,"at");
				fprintf(in,"%f	%d\r\n",op_sim_time(),node_address);
				fclose(in);
				
				op_td_set_int (pkptr, OPC_TDA_RA_PK_ACCEPT, OPC_FALSE);
				FOUT;
			}
		}else//接收节点的能量已耗尽，拒绝接收数据包
		{
			op_td_set_int (pkptr, OPC_TDA_RA_PK_ACCEPT, OPC_FALSE);
			FOUT;
		}
	}else
	{
		if(strcmp(node_type,"sensor")==0)
		{
			printf("在管道 13 阶段, 无法进行能量消耗计算，请检查sensor节点的 Model Attributes 名称在本阶段引用是否正确:\n\
					Energy Exhaust, Communication Range, Energy, E elec, E amp, Invalid Time Record File.\n");
		}
		if(strcmp(node_type,"sink")==0)
		{
			printf("在管道 13 阶段, 当前节点是sink节点，不需要进行能量计算.\n");
		}
	}
	
//自添加代码结束
	
	/* First, check whether this is a packet from a powerful jammer	*/
	/* rather than a WLAN packet.  If wlan_inoise has set a specific*/
	/* TDA for us, this indicates to not accept this jammer packet	*/
	/* Otherwise, accept the powerful jammer packet					*/
	if (op_pk_encap_flag_is_set (pkptr, OMSC_JAMMER_ENCAP_FLAG_INDEX))
		{
		/* Check for special TDA set by wlan_inoise, reject jammer 	*/
		/* packet accordingly										*/
		if (op_td_is_set (pkptr, OPC_TDA_RA_MAX_INDEX + 1))
			{
			op_td_set_int (pkptr, OPC_TDA_RA_PK_ACCEPT, OPC_FALSE); 
			FOUT;
			}
		
		op_td_set_int (pkptr, OPC_TDA_RA_PK_ACCEPT, OPC_TRUE);
		FOUT;
		}
	
	/* Otherwise, check the value of Accept field in the packet. If	*/
	/* it is an	interfering packet from a neighbor BSS with high	*/
	/* power, then the Accept field will be set to a special value.	*/
	op_pk_nfd_get_int32 (pkptr, "Accept", &current_status);

	/* Do not accept packets that were received when the node was	*/
	/* disabled or coming from an interferer node.					*/		
	if (current_status == WLANC_NEIGHBOR_BSS_PKT_REJECT || op_td_is_set (pkptr, OPC_TDA_RA_ND_FAIL))
		accept = OPC_FALSE;
	else
		{
		/* Obtain the error correction threshold of the receiver. 	*/
		ecc_thresh = op_td_get_dbl (pkptr, OPC_TDA_RA_ECC_THRESH);

		/* Obtain length of packet. 								*/
		pklen = op_pk_total_size_get (pkptr);

		/* Obtain number of errors in packet.						*/
		num_errs = op_td_get_int (pkptr, OPC_TDA_RA_NUM_ERRORS);
	
		/* Test if bit errors exceed threshold. 					*/
		if (pklen == 0)
			accept = OPC_TRUE;
		else
			accept = ((((double) num_errs) / pklen) <= ecc_thresh) ? OPC_TRUE : OPC_FALSE;
		}

	/* Place the flag indicating accept/reject in the data packet	*/
	/* control field. To save time, don't call the set function if	*/
	/* the field value is already set to a value that is same to 	*/
	/* what we are going to set.									*/
	if (accept != current_status)
		op_pk_nfd_set_int32 (pkptr, "Accept", accept);
    
	/* Force the simulation kernel to always accpet the packet. The	*/
	/* actual discarding of the packet will take place at the MAC	*/
	/* layer of the receiving node receiving this packet.      		*/
	op_td_set_int (pkptr, OPC_TDA_RA_PK_ACCEPT, OPC_TRUE);

	/* If this is an ERP-OFDM capable receiver (i.e. if its MAC		*/
	/* supports 802.11g PHY) and if this is an ERP-OFDM packet		*/
	/* successfully received, then add the "signal extension" delay	*/
	/* to the reception completion time and delay the arrival of	*/
	/* the packet to the MAC.										*/
	if (accept)
		{
		/* Access the receiver state information.					*/
	    rx_ch_objid   = op_td_get_int (pkptr, OPC_TDA_RA_RX_CH_OBJID);
		rx_state_ptr = (WlanT_Rx_State_Info *) op_ima_obj_state_get (rx_ch_objid);		
		if (rx_state_ptr->phy_tech == WlanC_11g_PHY)
			{
			/* Check the PHY used for packet's transmission.		*/
			op_pk_nfd_get_int32 (pkptr, "PHY Info", (int *) &rcvd_frame_phy_char);
			if (rcvd_frame_phy_char == WlanC_ERP_OFDM_11g)
				{
				/* Compute the reception completion time with		*/
				/* signal extension.								*/
				pk_reception_end_with_se = op_sim_time () + WLANC_11g_SIGNAL_EXTENSION;
				
				/* Update the reception end time information for	*/
				/* the receiver if necessary.						*/
				if (pk_reception_end_with_se > rx_state_ptr->rx_end_time)
					rx_state_ptr->rx_end_time = pk_reception_end_with_se;
				
				/* Delay the arrival of the packet to the MAC.		*/
				op_ima_obj_attr_set (rx_state_ptr->mac_strm_objid, "delay", WLANC_11g_SIGNAL_EXTENSION);
				}
			}
		
		/* In case of successfully received packets, check whether	*/
		/* congestion area monitoring is enabled. 					*/
		if (CONGESTION_AREAS_ENABLED)
			{
			char	node_name [OMSC_HNAME_MAX_LEN];
			char	port_name [OMSC_HNAME_MAX_LEN];
			Objid   port_objid; 
		
			if (rx_state_ptr->port_name_ptr == OPC_NIL) 
				{
				/* Obtain the receiver ID. 							*/
				port_objid = op_topo_parent (op_topo_parent (rx_ch_objid));
		
				/* Determine and store the port name. 				*/
				oms_tan_hname_get (port_objid, port_name);
				rx_state_ptr->port_name_ptr = (char *) op_prg_mem_alloc (sizeof (char) * (strlen (port_name) + 1));
				strcpy (rx_state_ptr->port_name_ptr, port_name); 
		
				/* Determine and store the node name. 				*/
				oms_tan_hname_get (op_topo_parent (port_objid), node_name);
				rx_state_ptr->node_name_ptr = (char *) op_prg_mem_alloc (sizeof (char) * (strlen (node_name) + 1));
				strcpy (rx_state_ptr->node_name_ptr, node_name); 
				}	
	
			/* Update the congestion area info. 					*/ 
			if (op_pk_encap_flag_is_set (pkptr, OMSC_BGUTIL_ENCAP_FLAG_INDEX))
				{
				oms_bgutil_state_info_update (pkptr, rx_state_ptr->data_rate, rx_state_ptr->port_name_ptr, 
					&rx_state_ptr->congestion_area,	OPC_NIL, OPC_NIL, &rx_state_ptr->routed_bgutil_state_ptr);
				}
			}		
		}

	FOUT
	}
