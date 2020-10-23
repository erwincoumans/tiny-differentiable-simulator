
//b3ReadWavFile is implemented based on code from the STK toolkit
//See https://github.com/thestk/stk
//Some improvement: the ticking data (b3WavTicker) is separate from wav file,
//This makes it possoble to play a single wav multiple times at the same time

#include "b3ReadWavFile.h"
#include "b3SwapUtils.h"
#include <math.h>

const unsigned long B3_SINT8 = 0x1;
const unsigned long B3_SINT16 = 0x2;
const unsigned long B3_SINT24 = 0x4;
const unsigned long B3_SINT32 = 0x8;
const unsigned long B3_FLOAT32 = 0x10;
const unsigned long B3_FLOAT64 = 0x20;

b3ReadWavFile::b3ReadWavFile()
{
	m_machineIsLittleEndian = 1;// b3MachineIsLittleEndian();
}
b3ReadWavFile::~b3ReadWavFile()
{
}

void b3ReadWavFile::normalize(double peak)
{
#if 0
	int i;
	double max = 0.0;

	for (i = 0; i < m_frames.size(); i++)
	{
		if (fabs(m_frames[i]) > max)
			max = (double)fabs((double)m_frames[i]);
	}

	if (max > 0.0)
	{
		max = 1.0 / max;
		max *= peak;
		for (i = 0; i < m_frames.size(); i++)
			m_frames[i] *= max;
	}
#endif
}

void b3ReadWavFile::interpolate(b3WavTicker* ticker, b3DataSource& dataSource, double speed, double volume, int size, myscalar* out0, myscalar* out1, int oIndex) const
{
	int iIndex = (int)ticker->time_;                        // integer part of index
	double output, alpha = ticker->time_ - (double)iIndex;  // fractional part of index

	iIndex = iIndex * channels_;
	
    if (dataType_ == B3_FLOAT32)
	{
	  float buf[4];
		if (dataSource.fseek( dataOffset_ + (iIndex * 4), B3_SEEK_SET) == -1)
			return;
		if (dataSource.fread(buf, 2*channels_ * 4, 1) != 1)
			return;
		double tmp0 = buf[0];
		double tmp1 = buf[1];
		if (alpha > 0.0)
		{
	  	tmp0 += (alpha * (buf[channels_] - tmp0));
	  	tmp1 += (alpha * (buf[channels_+1] - tmp1));
	  }
	  out0[oIndex]+=tmp0*volume;
	  out1[oIndex]+= (channels_ >1) ? tmp1*volume : tmp0 * volume;
	}
	
	if (dataType_ == B3_SINT8)
	{  // signed 8-bit data
		unsigned char buf[4];
		if (dataSource.fseek( dataOffset_ + iIndex, B3_SEEK_SET) == -1)
			return;
		if (dataSource.fread(buf, 2*channels_, 1) != 1)
			return;
		double gain = 1.0 / 128.0;
		
		double tmp0  = (buf[0] - 128)*gain;
		double tmp1  = (buf[1] - 128)*gain;
		if (alpha > 0.0)
		{
	  	tmp0 += (alpha * ((buf[channels_]-128)*gain - tmp0));
	  	tmp1 += (alpha * ((buf[channels_+1]-128)*gain - tmp1));
	  }
	  out0[oIndex]+=tmp0*volume;
	  out1[oIndex]+= (channels_ >1) ? tmp1*volume : tmp0 * volume;
	}

  if (dataType_ == B3_SINT24)
	{
		// 24-bit values are harder to import efficiently since there is
		// no native 24-bit type.  The following routine works but is much
		// less efficient than that used for the other data types.
		double buf[4];
		int temp;
		unsigned char *ptr = (unsigned char *)&temp;
		double gain = 1.0 / 2147483648.0;
		if (dataSource.fseek( dataOffset_ + (iIndex * 3), B3_SEEK_SET) == -1)
			return;
		for (int i = 0; i < 2*channels_; i++)
		{
			if (m_machineIsLittleEndian)
			{
				if (byteswap_)
				{
					if (dataSource.fread(ptr, 3, 1) != 1)
						return;
					temp &= 0x00ffffff;
					b3Swap32((unsigned char *)ptr);
				}
				else
				{
					if (dataSource.fread(ptr + 1, 3, 1) != 1)
						return;
					temp &= 0xffffff00;
				}
			}
			buf[i] = (double)temp * gain;  // "gain" also  includes 1 / 256 factor.
		}
		
		double tmp0 = buf[0];
		double tmp1 = buf[1];
		if (alpha > 0.0)
		{
	  	tmp0 += (alpha * (buf[channels_] - tmp0));
	  	tmp1 += (alpha * (buf[channels_+1] - tmp1));
	  }
	  out0[oIndex]+=tmp0*volume;
	  out1[oIndex]+= (channels_ >1) ? tmp1*volume : tmp0 * volume;
		
	}

	if (dataType_ == B3_SINT32)
	{
		int buf[4];
		if (dataSource.fseek( dataOffset_ + (iIndex * 4), B3_SEEK_SET) == -1)
			return;
		if (dataSource.fread(buf, 2*channels_ * 4, 1) != 1)
			return;
		double gain = 1.0 / 2147483648.0;
		
		double tmp0 = buf[0]*gain;
		double tmp1 = buf[1]*gain;
		
		if (alpha > 0.0)
		{
	  	tmp0 += (alpha * (buf[channels_]*gain - tmp0));
	  	tmp1 += (alpha * (buf[channels_+1]*gain - tmp1));
	  }
	  out0[oIndex]+=tmp0*volume;
	  out1[oIndex]+= (channels_ >1) ? tmp1*volume : tmp0 * volume;
	}

	if (dataType_ == B3_SINT16)
	{
		signed short int buf[4];
		if (dataSource.fseek( dataOffset_ + (iIndex * 2), B3_SEEK_SET) == -1)
			return;
		if (dataSource.fread(buf, 2*channels_ * 2, 1) != 1)
			return;
		double gain = 1.0 / 32768.0;
		
		double tmp0 = buf[0]*gain;
		double tmp1 = buf[1]*gain;
		if (alpha > 0.0)
		{
	  	tmp0 += (alpha * (buf[channels_]*gain - tmp0));
	  	tmp1 += (alpha * (buf[channels_+1]*gain - tmp1));
	  }
	  out0[oIndex]+=tmp0*volume;
	  out1[oIndex]+= (channels_ >1) ? tmp1*volume : tmp0 * volume;
	}
	
	
	if (dataType_ == B3_FLOAT64)
	{
		double buf[4];
		if (dataSource.fseek( dataOffset_ + (iIndex * 8), B3_SEEK_SET) == -1)
			return;
		if (dataSource.fread(buf, 2*channels_ * 8, 1) != 1)
			return;
	
	  double tmp0 = buf[0];
		double tmp1 = buf[1];
		if (alpha > 0.0)
		{
	  	tmp0 += (alpha * (buf[channels_] - tmp0));
	  	tmp1 += (alpha * (buf[channels_+1] - tmp1));
	  }
	  out0[oIndex]+=tmp0*volume;
	  out1[oIndex]+= (channels_ >1) ? tmp1*volume : tmp0 * volume;
  }

}


void b3ReadWavFile::tick(b3WavTicker *ticker, b3DataSource& dataSource, double speed, double volume, int size, myscalar* out0, myscalar* out1, int stride)
{
	if (ticker->finished_) 
	  return;
	if (ticker->time_ < ticker->starttime_ || ticker->time_ > ticker->endtime_)
	{
		ticker->finished_ = true;
		return;
	}

	for (int xx=0;xx<size;xx++)
	{
	  interpolate(ticker, dataSource, speed, volume, size, out0, out1, xx*stride);
		ticker->time_ += ticker->rate_*speed;
		if (ticker->time_ < ticker->starttime_ || ticker->time_ > ticker->endtime_)
		{
			ticker->finished_ = true;
			return;
		}
	}
}

void b3ReadWavFile::resize()
{
	//m_frames.resize(channels_ * m_numFrames);
}

b3WavTicker b3ReadWavFile::createWavTicker(double sampleRate)
{
	b3WavTicker ticker;
	ticker.lastFrame_.resize(this->channels_);
	ticker.time_ = 0;
	ticker.starttime_ = 0.;
	ticker.endtime_ = (double)(this->m_numFrames - 1.0);
	ticker.finished_ = false;
	ticker.rate_ = fileDataRate_ / sampleRate;
	ticker.speed = 1;
	return ticker;
}

bool b3ReadWavFile::getWavInfo(b3DataSource& dataSource)
{
	
	char header[12];
	if (dataSource.fread(&header, 4, 3) != 3)
		return false;
	bool res = false;

	if (!strncmp(header, "RIFF", 4) &&
		!strncmp(&header[8], "WAVE", 4))
		res = true;
	//getWavInfo( fileName );

	// Find "format" chunk ... it must come before the "data" chunk.
	char id[4];
	int chunkSize;
	if (dataSource.fread(&id, 4, 1) != 1)
		return false;
	while (strncmp(id, "fmt ", 4))
	{
		if (dataSource.fread(&chunkSize, 4, 1) != 1)
			return false;
		if (!m_machineIsLittleEndian)
		{
			b3Swap32((unsigned char *)&chunkSize);
		}
		if (dataSource.fseek( chunkSize, B3_SEEK_CUR) == -1)
			return false;
		if (dataSource.fread(&id, 4, 1) != 1)
			return false;
	}

	// Check that the data is not compressed.
	unsigned short format_tag;
	if (dataSource.fread(&chunkSize, 4, 1) != 1)
		return false;  // Read fmt chunk size.
	if (dataSource.fread(&format_tag, 2, 1) != 1)
		return false;
	if (!m_machineIsLittleEndian)
	{
		b3Swap16((unsigned char *)&format_tag);
		b3Swap32((unsigned char *)&chunkSize);
	}
	if (format_tag == 0xFFFE)
	{  // WAVE_FORMAT_EXTENSIBLE
		dataOffset_ = dataSource.ftell();
		if (dataSource.fseek( 14, B3_SEEK_CUR) == -1)
			return false;
		unsigned short extSize;
		if (dataSource.fread(&extSize, 2, 1) != 1)
			return false;
		if (!m_machineIsLittleEndian)
		{
			b3Swap16((unsigned char *)&extSize);
		}
		if (extSize == 0)
			return false;
		if (dataSource.fseek( 6, B3_SEEK_CUR) == -1)
			return false;
		if (dataSource.fread(&format_tag, 2, 1) != 1)
			return false;
		if (!m_machineIsLittleEndian)
		{
			b3Swap16((unsigned char *)&format_tag);
		}
		if (dataSource.fseek( dataOffset_, B3_SEEK_SET) == -1)
			return false;
	}
	if (format_tag != 1 && format_tag != 3)
	{  // PCM = 1, FLOAT = 3
		//  oStream_ << "FileRead: "<< fileName << " contains an unsupported data format type (" << format_tag << ").";
		return false;
	}

	// Get number of channels from the header.
	short int temp;
	if (dataSource.fread(&temp, 2, 1) != 1)
		return false;
	if (!m_machineIsLittleEndian)
	{
		b3Swap16((unsigned char *)&temp);
	}
	channels_ = (unsigned int)temp;

	// Get file sample rate from the header.
	int srate;
	if (dataSource.fread(&srate, 4, 1) != 1)
		return false;
	if (!m_machineIsLittleEndian)
	{
		b3Swap32((unsigned char *)&srate);
	}
	fileDataRate_ = (double)srate;

	// Determine the data type.
	dataType_ = 0;
	if (dataSource.fseek( 6, B3_SEEK_CUR) == -1)
		return false;  // Locate bits_per_sample info.
	if (dataSource.fread(&temp, 2, 1) != 1)
		return false;
	if (!m_machineIsLittleEndian)
	{
		b3Swap16((unsigned char *)&temp);
	}
	if (format_tag == 1)
	{
		if (temp == 8)
			dataType_ = B3_SINT8;
		else if (temp == 16)
			dataType_ = B3_SINT16;
		else if (temp == 24)
			dataType_ = B3_SINT24;
		else if (temp == 32)
			dataType_ = B3_SINT32;
	}
	else if (format_tag == 3)
	{
		if (temp == 32)
			dataType_ = B3_FLOAT32;
		else if (temp == 64)
			dataType_ = B3_FLOAT64;
	}
	if (dataType_ == 0)
	{
		//   oStream_ << "FileRead: " << temp << " bits per sample with data format " << format_tag << " are not supported (" << fileName << ").";
		return false;
	}

	// Jump over any remaining part of the "fmt" chunk.
	if (dataSource.fseek( chunkSize - 16, B3_SEEK_CUR) == -1)
		return false;

	// Find "data" chunk ... it must come after the "fmt" chunk.
	if (dataSource.fread(&id, 4, 1) != 1)
		return false;

	while (strncmp(id, "data", 4))
	{
		if (dataSource.fread(&chunkSize, 4, 1) != 1)
			return false;
		if (!m_machineIsLittleEndian)
		{
			b3Swap32((unsigned char *)&chunkSize);
		}
		chunkSize += chunkSize % 2;  // chunk sizes must be even
		if (dataSource.fseek( chunkSize, B3_SEEK_CUR) == -1)
			return false;
		if (dataSource.fread(&id, 4, 1) != 1)
			return false;
	}

	// Get length of data from the header.
	int bytes;
	if (dataSource.fread(&bytes, 4, 1) != 1)
		return false;
	if (!m_machineIsLittleEndian)
	{
		b3Swap32((unsigned char *)&bytes);
	}
	m_numFrames = bytes / temp / channels_;  // sample frames
	m_numFrames *= 8;                        // sample frames

	dataOffset_ = dataSource.ftell();
	byteswap_ = false;
	if (!m_machineIsLittleEndian)
	{
		byteswap_ = true;
	}
	wavFile_ = true;
	return true;
}

