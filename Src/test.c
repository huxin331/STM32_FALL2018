reconnect:
  /* Open socket 0 as TCP_SOCKET with port 5000 */
  if((retVal = socket(0, Sn_MR_TCP, 5000, 0)) == 0) {
	  /* Put socket in LISTEN mode. This means we are creating a TCP server */
	  if((retVal = listen(0)) == SOCK_OK) {
		  /* While socket is in LISTEN mode we wait for a remote connection */
		  while(sockStatus = getSn_SR(0) == SOCK_LISTEN)
			  HAL_Delay(100);
		  /* OK. Got a remote peer. Let's send a message to it */
		  while(1) {
			  /* If connection is ESTABLISHED with remote peer */
			  if(sockStatus = getSn_SR(0) == SOCK_ESTABLISHED) {
				  uint8_t remoteIP[4];
				  uint16_t remotePort;
				  /* Retrieving remote peer IP and port number */
				  getsockopt(0, SO_DESTIP, remoteIP);
				  getsockopt(0, SO_DESTPORT, (uint8_t*)&remotePort);
				  sprintf(msg, CONN_ESTABLISHED_MSG, remoteIP[0], remoteIP[1], remoteIP[2], remoteIP[3], remotePort);
				  PRINT_STR(msg);
				  /* Let's send a welcome message and closing socket */
				  if(retVal = send(0, GREETING_MSG, strlen(GREETING_MSG)) == (int16_t)strlen(GREETING_MSG))
					  PRINT_STR(SENT_MESSAGE_MSG);
				  else { /* Ops: something went wrong during data transfer */
					  sprintf(msg, WRONG_RETVAL_MSG, retVal);
					  PRINT_STR(msg);
				  }
				  break;
			  }
			  else { /* Something went wrong with remote peer, maybe the connection was closed unexpectedly */
				  sprintf(msg, WRONG_STATUS_MSG, sockStatus);
				  PRINT_STR(msg);
				  break;
			  }
		  }

	  } else /* Ops: socket not in LISTEN mode. Something went wrong */
		  PRINT_STR(LISTEN_ERR_MSG);
  } else { /* Can't open the socket. This means something is wrong with W5100 configuration: maybe SPI issue? */
	  sprintf(msg, WRONG_RETVAL_MSG, retVal);
	  PRINT_STR(msg);
  }

  /* We close the socket and start a connection again */
  disconnect(0);
  close(0);
  goto reconnect;
