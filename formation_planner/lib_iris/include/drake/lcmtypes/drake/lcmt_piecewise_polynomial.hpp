/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __drake_lcmt_piecewise_polynomial_hpp__
#define __drake_lcmt_piecewise_polynomial_hpp__

#include <vector>
#include "drake/lcmt_polynomial_matrix.hpp"

namespace drake
{

class lcmt_piecewise_polynomial
{
    public:
        int64_t    timestamp;

        int32_t    num_breaks;

        std::vector< double > breaks;

        int32_t    num_segments;

        std::vector< drake::lcmt_polynomial_matrix > polynomial_matrices;

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "lcmt_piecewise_polynomial"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int lcmt_piecewise_polynomial::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = (int64_t)getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int lcmt_piecewise_polynomial::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int lcmt_piecewise_polynomial::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t lcmt_piecewise_polynomial::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* lcmt_piecewise_polynomial::getTypeName()
{
    return "lcmt_piecewise_polynomial";
}

int lcmt_piecewise_polynomial::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_breaks, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->num_breaks > 0) {
        tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->breaks[0], this->num_breaks);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_segments, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->num_segments; a0++) {
        tlen = this->polynomial_matrices[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int lcmt_piecewise_polynomial::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_breaks, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->num_breaks) {
        this->breaks.resize(this->num_breaks);
        tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->breaks[0], this->num_breaks);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_segments, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->polynomial_matrices.resize(this->num_segments);
    for (int a0 = 0; a0 < this->num_segments; a0++) {
        tlen = this->polynomial_matrices[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int lcmt_piecewise_polynomial::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, this->num_breaks);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->num_segments; a0++) {
        enc_size += this->polynomial_matrices[a0]._getEncodedSizeNoHash();
    }
    return enc_size;
}

uint64_t lcmt_piecewise_polynomial::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == lcmt_piecewise_polynomial::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, (void*)lcmt_piecewise_polynomial::getHash };

    uint64_t hash = 0x6392ab0e78c32254LL +
         drake::lcmt_polynomial_matrix::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

}

#endif