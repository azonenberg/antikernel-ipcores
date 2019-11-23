/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file SSP21.svh
	@author Andrew D. Zonenberg
	@brief Protocol structures etc for SSP-21
 */

`ifndef SSP21_h
`define SSP21_h

typedef enum logic[7:0]
{
	REQUEST_HANDSHAKE_BEGIN	= 8'h0,
	REPLY_HANDSHAKE_BEGIN	= 8'h1,
	REPLY_HANDSHAKE_ERROR	= 8'h2,
	SESSION_DATA			= 8'h3
} ssp21_function_t;

typedef enum logic[1:0]
{
	SHARED_SECRET				= 2'h0,	//Pre-shared 256 bit secret
	PRESHARED_PUBLIC_KEYS		= 2'h1,	//Pre-shared X25519 asymmetric key pair
	INDUSTRIAL_CERTIFICATES		= 2'h2,	//PKI based (placeholder, not supported yet)
	QUANTUM_KEY_DISTRIBUTION	= 2'h3	//Quantum key distribution (placeholder, not supported yet)
} ssp21_handshakemode_t;

typedef enum logic[7:0]
{
	INCREMENT_LAST_RX		= 8'h0,
	GREATER_THAN_LAST_RX	= 8'h1
} ssp21_nonce_mode;

typedef enum logic[7:0]
{
	X25519					= 8'h0,
	NONCE					= 8'h1,
	NONE					= 8'h2
} ssp21_handshake_ephemeral_mode;

typedef enum logic[7:0]
{
	SHA256					= 8'h0
} ssp21_handshake_hash;

typedef enum logic[7:0]
{
	HKDF_SHA256				= 8'h0
} ssp21_handshake_kdf;

typedef enum logic[7:0]
{
	HMAC_SHA256_16	= 8'h0
} ssp21_session_crypto_mode;

typedef enum logic[7:0]
{
	BAD_MESSAGE_FORMAT					= 8'h0,
	UNSUPPORTED_VERSION					= 8'h1,
	UNSUPPORTED_HANDSHAKE_EPHEMERAL		= 8'h2,
	UNSUPPORTED_HANDSHAKE_HASH			= 8'h3,
	UNSUPPORTED_HANDSHAKE_KDF			= 8'h4,
	UNSUPPORTED_SESSION_MODE			= 8'h5,
	UNSUPPORTED_NONCE_MODE				= 8'h6,
	UNSUPPORTED_HANDSHAKE_MODE			= 8'h7,
	BAD_CERTIFICATE_FORMAT				= 8'h8,
	BAD_CERTIFICATE_CHAIN				= 8'h9,
	UNSUPPORTED_CERTIFICIATE_FEATURE	= 8'ha,
	AUTHENTICATION_ERROR				= 8'hb,
	NO_PRIOR_HANDSHAKE_BEGIN			= 8'hc,
	KEY_NOT_FOUND						= 8'hd,
	UNKNOWN								= 8'hff
} ssp21_handshake_error;

`endif
