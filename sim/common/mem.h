#pragma once

#include <cstdint>
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace vortex {
struct BadAddress {};

class MemDevice {
public:
  virtual ~MemDevice() {}
  virtual uint64_t size() const = 0;
  virtual void read(void *data, uint64_t addr, uint64_t size) = 0;
  virtual void write(const void *data, uint64_t addr, uint64_t size) = 0;
};

///////////////////////////////////////////////////////////////////////////////

class RamMemDevice : public MemDevice {
public:
  RamMemDevice(uint64_t size, uint32_t wordSize);
  RamMemDevice(const char *filename, uint32_t wordSize);
  ~RamMemDevice() {}

  void read(void *data, uint64_t addr, uint64_t size) override;  
  void write(const void *data, uint64_t addr, uint64_t size) override;

  virtual uint64_t size() const {
    return contents_.size();
  };

protected:
  std::vector<uint8_t> contents_;
  uint32_t wordSize_;
};

///////////////////////////////////////////////////////////////////////////////

class RomMemDevice : public RamMemDevice {
public:
  RomMemDevice(const char *filename, uint32_t wordSize)
    : RamMemDevice(filename, wordSize) 
  {}

  RomMemDevice(uint64_t size, uint32_t wordSize)
    : RamMemDevice(size, wordSize) 
  {}
  
  ~RomMemDevice();

  void write(const void *data, uint64_t addr, uint64_t size) override;
};

///////////////////////////////////////////////////////////////////////////////

class RAM : public MemDevice {
public:
  
  RAM(uint32_t page_size);
  ~RAM();

  void clear();

  uint64_t size() const override;

  void read(void *data, uint64_t addr, uint64_t size) override;  
  void write(const void *data, uint64_t addr, uint64_t size) override;

  void loadBinImage(const char* filename, uint64_t destination);
  void loadHexImage(const char* filename);

  uint8_t& operator[](uint64_t address) {
    return *this->get(address);
  }

  const uint8_t& operator[](uint64_t address) const {
    return *this->get(address);
  }

  uint64_t getPageRootTable();
  uint64_t getFirstFreeTable();

private:

  uint8_t *get(uint64_t address) const;
  void initializeRootTable();

  uint64_t size_;
  uint32_t page_bits_;  
  mutable std::unordered_map<uint64_t, uint8_t*> pages_;
  mutable uint8_t* last_page_;
  mutable uint64_t last_page_index_;
  uint64_t rootPageTableNumber_ = -1;
  bool isPageRootTableInitialized_ = false;
};


class VirtualDevice: public RAM{
  public:
  VirtualDevice(uint32_t page_size): RAM(page_size){

  }
};
///////////////////////////////////////////////////////////////////////////////


class MemoryUnit {
public:
  
  struct PageFault {
    PageFault(uint64_t a, bool nf)
      : faultAddr(a)
      , notFound(nf) 
    {}
    uint64_t faultAddr;
    bool notFound;
  };

  struct TlbMiss
  {
      TlbMiss(uint64_t a, bool nf)
      : faultAddr(a)
      , notFound(nf) 
    {}
    uint64_t faultAddr;
    bool notFound;/* data */
  };
  

  MemoryUnit(uint64_t pageSize, uint64_t addrBytes, bool disableVm = false);
  MemoryUnit(uint64_t pageSize, uint64_t addrBytes, uint64_t rootAddress);
  void attach(MemDevice &m, uint64_t start, uint64_t end);
  void read(void *data, uint64_t addr, uint64_t size, bool sup);  
  void write(const void *data, uint64_t addr, uint64_t size, bool sup);
  void attachRAM(RAM &ram, uint64_t start, uint64_t end);
  void attachVirtualDevice(VirtualDevice &virtualDevice);
  void requestVirtualPage(uint8_t* data,uint64_t virtualAddr);

  void tlbAdd(uint64_t virt, uint64_t phys, uint32_t flags);
  void tlbRm();
  void tlbFlush() {
    tlb_.clear();
  }
private:

  class ADecoder {
  public:
    ADecoder() {}
    
    void read(void *data, uint64_t addr, uint64_t size);
    void write(const void *data, uint64_t addr, uint64_t size);
    
    void map(uint64_t start, uint64_t end, MemDevice &md);

  private:

    struct mem_accessor_t {
      MemDevice* md;
      uint64_t addr;
    };
    
    struct entry_t {
      MemDevice *md;
      uint64_t      start;
      uint64_t      end;        
    };

    bool lookup(uint64_t a, uint32_t wordSize, mem_accessor_t*);

    std::vector<entry_t> entries_;
  };

  struct TLBEntry {
    TLBEntry() {}
    TLBEntry(uint32_t pfn, uint32_t flags)
      : pfn(pfn)
      , flags(flags) 
    {}
    uint32_t pfn;
    uint32_t flags;
    bool isAccessBitSet;
  };

  class TablePageEntry
  {
      public:
      TablePageEntry(uint64_t address)
      {
          bytes_ = address;
      }

      bool isValid(){
          return getBit(VIndex);
      }

      bool isExecutable(){
        return getBit(XIndex);
      }

      bool isReadable(){
        return getBit(RIndex);
      }

      bool isWritable(){
        return getBit(WIndex);
      }

      uint64_t getNextTableAddress()
      {
          return (bytes_ >> (DIndex+1)) & (((uint64_t)1 << (PageNumberLength+(uint64_t)1)) - (uint64_t)1); 
      }

      private:

      bool getBit(int bitNumber){
          return (bytes_ >> bitNumber) & 1;
      }

      static const uint64_t PageNumberLength = 44;
      static const int DIndex = 7;
      static const int AIndex = 6;
      static const int GIndex = 5;
      static const int UIndex = 4;
      static const int XIndex = 3;
      static const int WIndex = 2;
      static const int RIndex = 1;
      static const int VIndex = 0; 

      uint64_t bytes_;
    };

    class SV39VirtualAddress
    {
    public:
    SV39VirtualAddress(uint64_t address){
        address_ = address;
    }

    uint64_t getOffsetForLevel(int level)
    {
        uint64_t index = level * OffsetInPageLength + PageOffsetLength;
        uint64_t offset = (address_ >>index) & ((1 << OffsetInPageLength) - 1);
        return offset;
    }

    uint64_t getOffset(){
        uint64_t offset = address_ & ((1 << PageOffsetLength) - 1);
        return offset;
    }

    uint64_t getVirtualAddress(){
      return (address_ >> PageOffsetLength) & ((1<<PageOffsetLength) - 1); 
    }


    private:
    uint64_t address_;
    static const int LevelsCount =3;
    static const int PageOffsetLength = 12;
    static const int OffsetInPageLength = 9; 
    };

  uint64_t translateVirtualToPhysical(uint64_t vAddr, uint32_t flagMask);
  TLBEntry tlbLookup(uint64_t vAddr, uint32_t flagMask);
  void handleTblMiss(uint64_t vAddr);
  void handlePageTableMiss(uint64_t vAddr);
  void updateTLBIfNeeded();
  uint64_t allocate_translation_table();

  void* requestPage(uint64_t address){
    void*data;
    ram_->read(data, address, pageSize_);
  }

  std::unordered_map<uint64_t, TLBEntry> tlb_;
  uint64_t pageSize_;
  uint64_t addrBytes_;
  ADecoder decoder_;  
  bool disableVM_;
  uint64_t rootPageAddress_; 
  uint64_t pageLevels_ = 3;
  uint64_t levelPagesInMemory_ = 2;
  RAM* ram_;
  int memoryAccessCount_ =0;
  VirtualDevice* virtualDevice_;

  static const int maxPageTableEntriesCount = 512;
  static const int RefreshTblRate = 20;
};


///////////////////////////////////////////////////////////////////////////////

} // namespace vortex