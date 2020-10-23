#ifndef B3_READ_WAV_FILE_H
#define B3_READ_WAV_FILE_H

#include <vector>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#define B3_SEEK_CUR    11
#define B3_SEEK_END    12
#define B3_SEEK_SET    10

typedef double myscalar;

struct b3DataSource
{
	virtual long  ftell() = 0;
	
	virtual size_t fread(void* _Buffer, size_t _ElementSize, size_t _ElementCount) = 0;

	virtual int fseek(long  _Offset, int   _Origin) = 0;
	
	virtual int size() = 0;
};

struct FileDataSource : public b3DataSource
{
	FileDataSource(const char* fileName)
	{
		m_fd = fopen(fileName, "rb");
	}
	virtual ~FileDataSource()
	{
		if (m_fd)
			fclose(m_fd);
	}
	FILE* m_fd;

	virtual long  ftell()
	{
		return ::ftell(m_fd);
	}
	virtual size_t fread(void* _Buffer, size_t _ElementSize, size_t _ElementCount)
	{
		return ::fread(_Buffer, _ElementSize, _ElementCount, m_fd);
	}

	virtual int fseek(long  _Offset, int   _OriginOrg)
	{
		int _Origin = 0;
		switch (_OriginOrg)
		{
		case B3_SEEK_CUR:
		{
			_Origin = SEEK_CUR;
			break;
		}
		case B3_SEEK_SET:
		{
			_Origin = SEEK_SET;
			break;
		}
		default:
		{
		}
		}
		return ::fseek(m_fd, _Offset, _Origin);
	}
	virtual int size()
	{
		int size = 0;
		if (::fseek(m_fd, 0, SEEK_END) || (size = ::ftell(m_fd)) == EOF || ::fseek(m_fd, 0, SEEK_SET))
		{
			assert(0);
			size = -1;
		}
		return size;
	}
};

struct MemoryDataSource : public b3DataSource
{

  MemoryDataSource()
  :m_data(),
  m_numBytes(0),
	m_currentAddress(0)
  {
    
  }
	MemoryDataSource(const char* data, int numBytes)
		:m_data(data),
		m_numBytes(numBytes),
		m_currentAddress(0)
	{
	}

	const char* m_data;
	int m_numBytes;
	int m_currentAddress;

	virtual long  ftell()
	{
		return m_currentAddress;
	}

	virtual size_t fread(void* _BufferOrg, size_t _ElementSize, size_t _ElementCount)
	{
		char* _Buffer = (char*)_BufferOrg;
		int writeIndex = 0;
		int i = 0;
		for (i = 0; i < _ElementCount; i++)
		{
			for (int j = 0; j < _ElementSize; j++)
			{
				if (m_currentAddress < m_numBytes)
				{
					_Buffer[writeIndex++] = m_data[m_currentAddress++];
				}
				else
				{
					break;
					assert(0);
				}
			}
		}
		return i;
	}


	virtual int fseek(long  _Offset, int   _Origin)
	{
		switch (_Origin)
		{
			case B3_SEEK_CUR:
			{
				m_currentAddress += _Offset;
				break;
			}
			case B3_SEEK_SET:
			{
				m_currentAddress = _Offset;
				break;
			}
			default:
			{
			}
		}
		int result = -1;
		if (m_currentAddress >= 0 && m_currentAddress < m_numBytes)
			result = 0;
		return result;
	}

	virtual int size()
	{
		return m_numBytes;
	}
};




struct b3WavTicker
{
	std::vector<double> lastFrame_;
	bool finished_;
	double time_;
	double starttime_;
	double endtime_;
	double rate_;
	double speed;
	int wavindex;
	double env_volume()
	{
		double frac = 1. - (time_ - starttime_) / (endtime_ - starttime_);
		return frac;
	}

	double env_volume2()
	{
		double frac = (time_ - starttime_) / (endtime_ - starttime_);
		if (frac > 0.5)
			return 1. - frac;
		return frac;
	}
};

class b3ReadWavFile
{
	bool byteswap_;
	bool wavFile_;
	unsigned long m_numFrames;
	unsigned long dataType_;
	double fileDataRate_;
	
	unsigned long dataOffset_;
	unsigned int channels_;
	bool m_machineIsLittleEndian;

public:
	b3ReadWavFile();
	virtual ~b3ReadWavFile();

	//btAlignedObjectArray<double> m_frames;

	bool getWavInfo(b3DataSource& dataSource);

	void normalize(double peak);

	void interpolate(b3WavTicker *ticker, b3DataSource& dataSource, double speed, double volume, int size, myscalar* out0, myscalar* out1, int oIndex) const;
	void tick(b3WavTicker *ticker, b3DataSource& dataSource, double speed, double volume, int size, myscalar* out0, myscalar* out1, int stride);
	
	void resize();

	b3WavTicker createWavTicker(double sampleRate);

	int getNumFrames() const
	{
		return m_numFrames;
	}
	
	double getFileDataRate()
	{
	  return fileDataRate_;
	}
};

#endif  //B3_READ_WAV_FILE_H
