#include "mem.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <assert.h>
#include "util.h"
#include <algorithm>

using namespace vortex;

RamMemDevice::RamMemDevice(const char *filename, uint32_t wordSize) 
  : wordSize_(wordSize) {
  std::ifstream input(filename);

  if (!input) {
    std::cout << "Error reading file \"" << filename << "\" into RamMemDevice.\n";
    std::abort();
  }

  do {
    contents_.push_back(input.get());
  } while (input);

  while (contents_.size() & (wordSize-1))
    contents_.push_back(0x00);
}

RamMemDevice::RamMemDevice(uint64_t size, uint32_t wordSize)
  : contents_(size) 
  , wordSize_(wordSize)
{}

void RamMemDevice::read(void *data, uint64_t addr, uint64_t size) {
  auto addr_end = addr + size;
  if ((addr & (wordSize_-1))
   || (addr_end & (wordSize_-1)) 
   || (addr_end <= contents_.size())) {
    std::cout << "lookup of 0x" << std::hex << (addr_end-1) << " failed.\n";
    throw BadAddress();
  }  
  
  const uint8_t *s = contents_.data() + addr;
  for (uint8_t *d = (uint8_t*)data, *de = d + size; d != de;) {
    *d++ = *s++;
  }
}

void RamMemDevice::write(const void *data, uint64_t addr, uint64_t size) {
  auto addr_end = addr + size;
  if ((addr & (wordSize_-1))
   || (addr_end & (wordSize_-1)) 
   || (addr_end <= contents_.size())) {
    std::cout << "lookup of 0x" << std::hex << (addr_end-1) << " failed.\n";
    throw BadAddress();
  }

  const uint8_t *s = (const uint8_t*)data;
  for (uint8_t *d = contents_.data() + addr, *de = d + size; d != de;) {
    *d++ = *s++;
  }
}

///////////////////////////////////////////////////////////////////////////////

void RomMemDevice::write(const void* /*data*/, uint64_t /*addr*/, uint64_t /*size*/) {
  std::cout << "attempt to write to ROM.\n";
  std::abort();
}

///////////////////////////////////////////////////////////////////////////////

bool MemoryUnit::ADecoder::lookup(uint64_t a, uint32_t wordSize, mem_accessor_t* ma) {
  uint64_t e = a + (wordSize - 1);
  assert(e >= a);
  for (auto iter = entries_.rbegin(), iterE = entries_.rend(); iter != iterE; ++iter) {
    if (a >= iter->start && e <= iter->end) {
      ma->md   = iter->md;
      ma->addr = a - iter->start;
      return true;
    }
  }
  return false;
}

void MemoryUnit::ADecoder::map(uint64_t a, uint64_t e, MemDevice &m) {
  assert(e >= a);
  entry_t entry{&m, a, e};
  entries_.emplace_back(entry);
}

void MemoryUnit::ADecoder::read(void *data, uint64_t addr, uint64_t size) {
  mem_accessor_t ma;
  if (!this->lookup(addr, size, &ma)) {
    std::cout << "lookup of 0x" << std::hex << addr << " failed.\n";
    throw BadAddress();
  }      
  ma.md->read(data, ma.addr, size);
}

void MemoryUnit::ADecoder::write(const void *data, uint64_t addr, uint64_t size) {
  mem_accessor_t ma;
  if (!this->lookup(addr, size, &ma)) {
    std::cout << "lookup of 0x" << std::hex << addr << " failed.\n";
    throw BadAddress();
  }
  ma.md->write(data, ma.addr, size);
}

///////////////////////////////////////////////////////////////////////////////

MemoryUnit::MemoryUnit(uint64_t pageSize, uint64_t addrBytes, bool disableVm)
  : pageSize_(pageSize)
  , addrBytes_(addrBytes)
  , disableVM_(disableVm) {
  if (!disableVm) {
    tlb_[0] = TLBEntry(0, 077);
  }

  ram_ = nullptr;
}

MemoryUnit::MemoryUnit(uint64_t pageSize, uint64_t addrBytes, uint64_t rootPageNumber)
  : pageSize_(pageSize)
  , addrBytes_(addrBytes)
  , disableVM_(false) {
    tlb_[0] = TLBEntry(0, 077);
}

void MemoryUnit::attach(MemDevice &m, uint64_t start, uint64_t end) {
  decoder_.map(start, end, m);
}

MemoryUnit::TLBEntry MemoryUnit::tlbLookup(uint64_t vAddr, uint32_t flagMask) {
  auto iter = tlb_.find(vAddr / pageSize_);
  if (iter != tlb_.end()) {
    if (iter->second.flags & flagMask)
      return iter->second;
    else {
      throw TlbMiss(vAddr, false);
    }
  } else {
    throw TlbMiss(vAddr, true);
  }
}

void MemoryUnit::read(void *data, uint64_t addr, uint64_t size, bool sup) {
  updateTLBIfNeeded();
  uint64_t pAddr;
  if (disableVM_) {
    pAddr = addr;
  } else {
    uint32_t flagMask = sup ? 8 : 1;
    try{
      TLBEntry t = this->tlbLookup(addr, flagMask);  
      pAddr = t.pfn * pageSize_ + addr % pageSize_;
    }
    catch(TlbMiss e){
      if(e.notFound){
        pAddr = translateVirtualToPhysical(addr, flagMask);
      }
      else{
        throw e;
      }
    }
  }

  return decoder_.read(data, pAddr, size);
}

uint64_t MemoryUnit::translateVirtualToPhysical(uint64_t vAddr,  uint32_t flagMask){
  SV39VirtualAddress virtualAddressObj(vAddr);
  uint64_t currentPageNumber = this->rootPageAddress_;
  uint64_t address;
  uint64_t pte;
  uint64_t pAddr;
  bool shouldHandlePageFault = false;
  for(int i  = pageLevels_ - 1;i >=0; i-- ){  
    address = (currentPageNumber << 12) + (virtualAddressObj.getOffsetForLevel(i))*sizeof(uint64_t);
    this->ram_->read(&pte, address, sizeof(uint64_t));
    TablePageEntry tableEntry(pte);
    if(tableEntry.isValid()){
      if(tableEntry.isReadable()
          || tableEntry.isExecutable()){

            uint64_t pageOffset = virtualAddressObj.getOffset();
            uint64_t tableEntryAddress = tableEntry.getNextTableAddress();
            pAddr = (tableEntryAddress << 12) | pageOffset; 
         }
         else{
            currentPageNumber = tableEntry.getNextTableAddress();
         }
    }else{

      uint64_t newTableAddress;
  
      uint16_t flags=1;
      if(i==0){
        flags =  7; 
        uint64_t pageOffset = virtualAddressObj.getOffset();
        uint64_t newRAMPageNumber = ram_->getFirstFreeTable();
        uint64_t newPageAddress = newRAMPageNumber << 12;
        uint8_t* virtualPageData = new uint8_t[pageSize_];
        requestVirtualPage(virtualPageData, vAddr);
        ram_->write(virtualPageData, newPageAddress, pageSize_);
        pAddr = (newPageAddress) | pageOffset; 
        newTableAddress = newRAMPageNumber;
      } else{
        newTableAddress = allocate_translation_table();
      }

      uint64_t newTableEntry = (newTableAddress << 8)  | flags;
      this->ram_->write(&newTableEntry, address, sizeof(uint64_t));
      currentPageNumber = newTableAddress;
    }
  }

  return pAddr;
}

void MemoryUnit::requestVirtualPage(uint8_t* data, uint64_t virtualAddress){
  virtualDevice_->read(data, virtualAddress, pageSize_);  
}

uint64_t MemoryUnit::allocate_translation_table(){
  uint64_t newPageAddress = this->ram_->getFirstFreeTable();
  uint8_t zeroByte = 0;
  uint64_t currentPageAddress = newPageAddress << 12;
  for(int i=0;i< pageSize_;i++){
    this->ram_->write(&zeroByte, currentPageAddress, sizeof(uint8_t));
    currentPageAddress+= 1;
  }

  return newPageAddress;
}


void MemoryUnit::write(const void *data, uint64_t addr, uint64_t size, bool sup) {
  updateTLBIfNeeded();
  uint64_t pAddr;
  if (disableVM_) {
    pAddr = addr;
  } else {
    uint32_t flagMask = sup ? 16 : 2;
    try{
      TLBEntry t = tlbLookup(addr, flagMask);
      pAddr = t.pfn * pageSize_ + addr % pageSize_;
    }
    catch(PageFault e){
      if(e.notFound){
        pAddr = translateVirtualToPhysical(addr, flagMask);
      }
    }
  }

  decoder_.write(data, pAddr, size);
}

void MemoryUnit::tlbAdd(uint64_t virt, uint64_t phys, uint32_t flags) {
  SV39VirtualAddress vAddr(virt);
  tlb_[vAddr.getVirtualAddress()] = TLBEntry(phys / pageSize_, flags);
}

void MemoryUnit::tlbRm() {
  for(auto& it: tlb_){
    if(!it.second.isAccessBitSet){
      tlb_.erase(it.first);
    }
  }
}

void  MemoryUnit::attachRAM(RAM &ram, uint64_t start, uint64_t end){
  this->attach(ram, start,end);
  rootPageAddress_ = ram.getPageRootTable();
  this->ram_ = &ram;
}

void MemoryUnit::attachVirtualDevice(VirtualDevice &virtualDevice){
  this->virtualDevice_ = &virtualDevice;
}

void MemoryUnit::updateTLBIfNeeded(){
  if(memoryAccessCount_ == RefreshTblRate){
    memoryAccessCount_ = 0;
  }else{
    return;
  }

  if(tlb_.size() == maxPageTableEntriesCount){
    for(auto& it: tlb_){
      it.second.isAccessBitSet = false;
    }
  }
}
///////////////////////////////////////////////////////////////////////////////

RAM::RAM(uint32_t page_size) 
  : size_(0)
  , page_bits_(log2ceil(page_size))
  , last_page_(nullptr)
  , last_page_index_(0) {    
   assert(ispow2(page_size));
}

RAM::~RAM() {
  this->clear();
}

void RAM::clear() {
  for (auto& page : pages_) {
    delete[] page.second;
  }
}

uint64_t RAM::size() const {
  return uint64_t(pages_.size()) << page_bits_;
}

uint8_t *RAM::get(uint64_t address) const {
  uint32_t page_size   = 1 << page_bits_;  
  uint32_t page_offset = address & (page_size - 1);
  uint64_t page_index  = address >> page_bits_;

  uint8_t* page;
  if (last_page_ && last_page_index_ == page_index) {
    page = last_page_;
  } else {
    auto it = pages_.find(page_index);
    if (it != pages_.end()) {
      page = it->second;
    } else {
      uint8_t *ptr = new uint8_t[page_size];
      // set uninitialized data to "baadf00d"
      for (uint32_t i = 0; i < page_size; ++i) {
        ptr[i] = (0xbaadf00d >> ((i & 0x3) * 8)) & 0xff;
      }
      pages_.emplace(page_index, ptr);
      page = ptr; 
    }
    last_page_ = page;
    last_page_index_ = page_index;
  }

  return page + page_offset;
}

void RAM::read(void *data, uint64_t addr, uint64_t size) {
  uint8_t* d = (uint8_t*)data;
  for (uint64_t i = 0; i < size; i++) {
    d[i] = *this->get(addr + i);
  }
}

void RAM::write(const void *data, uint64_t addr, uint64_t size) {
  const uint8_t* d = (const uint8_t*)data;
  for (uint64_t i = 0; i < size; i++) {
    *this->get(addr + i) = d[i];
  }
}

void RAM::loadBinImage(const char* filename, uint64_t destination) {
  std::ifstream ifs(filename);
  if (!ifs) {
    std::cout << "error: " << filename << " not found" << std::endl;
  }

  ifs.seekg(0, ifs.end);
  size_t size = ifs.tellg();
  std::vector<uint8_t> content(size);
  ifs.seekg(0, ifs.beg);
  ifs.read((char*)content.data(), size);

  this->clear();
  this->write(content.data(), destination, size);
}

uint64_t RAM::getFirstFreeTable(){
  uint64_t* existingIndexes  = new uint64_t[pages_.size()];
  int pageNumber = 0;
  for(auto& it: pages_){
    existingIndexes[pageNumber] = it.first;
    pageNumber++;
  }

  // To get root table first free table is used 
  auto notUsedTableNumber = *std::max_element(existingIndexes, existingIndexes + pages_.size()) + 1 ;
  uint64_t address = notUsedTableNumber >> page_bits_;
  // intialize
  this->get(address);
  return notUsedTableNumber;
}

void RAM::initializeRootTable(){
  if(pages_.size() ==0){
    rootPageTableNumber_ = 0;
  }
  else{
    rootPageTableNumber_ = getFirstFreeTable();    
  } 
  uint8_t* data = this->get(rootPageTableNumber_ << 12);
    for(int i=0;i< (1 << page_bits_);i++){
      data[i] = 0;
  }
}

uint64_t RAM::getPageRootTable(){
  if(!isPageRootTableInitialized_){
    initializeRootTable();
  }

  return rootPageTableNumber_;
}

void RAM::loadHexImage(const char* filename) {
  auto hti = [&](char c)->uint32_t {
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    return c - '0';
  };

  auto hToI = [&](const char *c, uint32_t size)->uint32_t {
    uint32_t value = 0;
    for (uint32_t i = 0; i < size; i++) {
      value += hti(c[i]) << ((size - i - 1) * 4);
    }
    return value;
  };

  std::ifstream ifs(filename);
  if (!ifs) {
    std::cout << "error: " << filename << " not found" << std::endl;
  }

  ifs.seekg(0, ifs.end);
  size_t size = ifs.tellg();
  std::vector<char> content(size);
  ifs.seekg(0, ifs.beg);
  ifs.read(content.data(), size);

  uint32_t offset = 0;
  char *line = content.data();

  this->clear();

  while (true) {
    if (line[0] == ':') {
      uint32_t byteCount = hToI(line + 1, 2);
      uint32_t nextAddr = hToI(line + 3, 4) + offset;
      uint32_t key = hToI(line + 7, 2);
      switch (key) {
      case 0:
        for (uint32_t i = 0; i < byteCount; i++) {
          uint32_t addr  = nextAddr + i;
          uint32_t value = hToI(line + 9 + i * 2, 2);
          *this->get(addr) = value;
        }
        break;
      case 2:
        offset = hToI(line + 9, 4) << 4;
        break;
      case 4:
        offset = hToI(line + 9, 4) << 16;
        break;
      default:
        break;
      }
    }
    while (*line != '\n' && size != 0) {
      ++line;
      --size;
    }
    if (size <= 1)
      break;
    ++line;
    --size;
  }
}