#include "fishhook.h"

#include <dlfcn.h>
#include <stdbool.h>

#import <mach-o/dyld.h>
#import <mach-o/loader.h>
#import <mach-o/nlist.h>

#ifdef __LP64__
typedef struct mach_header_64 mach_header_t;
typedef struct segment_command_64 segment_command_t;
typedef struct load_command load_command_t;
typedef struct encryption_info_command_64 encryption_info_command_t;
typedef struct section_64 section_t;
typedef struct nlist_64 nlist_t;
#define LC_SEGMENT_ARCH_DEPENDENT LC_SEGMENT_64
#else
typedef struct mach_header mach_header_t;
typedef struct segment_command segment_command_t;
typedef struct load_command load_command_t;
typedef struct encryption_info_command encryption_info_command_t;
typedef struct section section_t;
typedef struct nlist nlist_t;
#define LC_SEGMENT_ARCH_DEPENDENT LC_SEGMENT
#endif

#ifndef SEG_DATA_CONST
#define SEG_DATA_CONST "__DATA_CONST"
#endif

struct rebind_cached_symbol {
  const char *name;
  void *replacement;
  void **replaced;
};

struct rebind_cached_symbol_list {
  struct rebind_cached_symbol *rebindings;
  size_t rebindings_nel;
  struct rebind_cached_symbol_list *next;
};

static struct rebind_cached_symbol_list *_rebindings_head;

static int rebind_symbols_for_image(struct rebind_cached_symbol_list *rebindings,
                                    const mach_header_t *header,
                                    intptr_t slide) {
  Dl_info info;
  if (dladdr(header, &info) == 0) {
    return 0;
  }

  segment_command_t *cur_seg_cmd;
  segment_command_t *linkedit_segment = NULL;
  struct symtab_command *symtab_cmd = NULL;
  struct dysymtab_command *dysymtab_cmd = NULL;

  uintptr_t cur = (uintptr_t)header + sizeof(mach_header_t);
  for (uint32_t i = 0; i < header->ncmds; i++, cur += cur_seg_cmd->cmdsize) {
    cur_seg_cmd = (segment_command_t *)cur;
    if (cur_seg_cmd->cmd == LC_SEGMENT_ARCH_DEPENDENT) {
      if (strcmp(cur_seg_cmd->segname, SEG_LINKEDIT) == 0) {
        linkedit_segment = cur_seg_cmd;
      }
    } else if (cur_seg_cmd->cmd == LC_SYMTAB) {
      symtab_cmd = (struct symtab_command *)cur_seg_cmd;
    } else if (cur_seg_cmd->cmd == LC_DYSYMTAB) {
      dysymtab_cmd = (struct dysymtab_command *)cur_seg_cmd;
    }
  }

  if (!linkedit_segment || !symtab_cmd || !dysymtab_cmd) {
    return 0;
  }

  uintptr_t linkedit_base = (uintptr_t)slide + linkedit_segment->vmaddr - linkedit_segment->fileoff;
  nlist_t *symtab = (nlist_t *)(linkedit_base + symtab_cmd->symoff);
  char *strtab = (char *)(linkedit_base + symtab_cmd->stroff);
  uint32_t *indirect_symtab = (uint32_t *)(linkedit_base + dysymtab_cmd->indirectsymoff);

  cur = (uintptr_t)header + sizeof(mach_header_t);
  for (uint32_t i = 0; i < header->ncmds; i++, cur += cur_seg_cmd->cmdsize) {
    cur_seg_cmd = (segment_command_t *)cur;
    if (cur_seg_cmd->cmd == LC_SEGMENT_ARCH_DEPENDENT) {
      if (strcmp(cur_seg_cmd->segname, SEG_DATA) != 0 &&
          strcmp(cur_seg_cmd->segname, SEG_DATA_CONST) != 0) {
        continue;
      }
      for (uint32_t j = 0; j < cur_seg_cmd->nsects; j++) {
        section_t *sect = (section_t *)(cur + sizeof(segment_command_t)) + j;
        if ((sect->flags & SECTION_TYPE) == S_LAZY_SYMBOL_POINTERS ||
            (sect->flags & SECTION_TYPE) == S_NON_LAZY_SYMBOL_POINTERS) {
          
          uint32_t *indirect_symbol_indices = indirect_symtab + sect->reserved1;
          void **indirect_symbol_bindings = (void **)((uintptr_t)slide + sect->addr);

          for (uint32_t k = 0; k < sect->size / sizeof(void *); k++) {
            uint32_t symtab_index = indirect_symbol_indices[k];
            if (symtab_index == INDIRECT_SYMBOL_ABS || symtab_index == INDIRECT_SYMBOL_LOCAL) {
              continue;
            }
            uint32_t strtab_offset = symtab[symtab_index].n_un.n_strx;
            char *symbol_name = strtab + strtab_offset;
            bool symbol_has_leading_underscore = symbol_name[0] == '_';

            struct rebind_cached_symbol_list *cur_rebindings = rebindings;
            while (cur_rebindings) {
              for (size_t m = 0; size_t_m_less = m < cur_rebindings->rebindings_nel; m++) {
                if (strcmp(&symbol_name[symbol_has_leading_underscore ? 1 : 0], cur_rebindings->rebindings[m].name) == 0) {
                  if (cur_rebindings->rebindings[m].replaced != NULL &&
                      indirect_symbol_bindings[k] != cur_rebindings->rebindings[m].replacement) {
                    *(cur_rebindings->rebindings[m].replaced) = indirect_symbol_bindings[k];
                  }
                  indirect_symbol_bindings[k] = cur_rebindings->rebindings[m].replacement;
                  goto symbol_loop_continue;
                }
              }
              cur_rebindings = cur_rebindings->next;
            }
          symbol_loop_continue:;
          }
        }
      }
    }
  }
  return 0;
}

static void _rebind_symbols_for_image(const struct mach_header *header, intptr_t slide) {
  rebind_symbols_for_image(_rebindings_head, (const mach_header_t *)header, slide);
}

__attribute__((visibility("hidden")))
int rebind_symbols_image(void *header,
                         intptr_t slide,
                         struct rebinding rebindings[],
                         size_t rebindings_nel) {
  struct rebind_cached_symbol_list cached_rebindings;
  cached_rebindings.rebindings = (struct rebind_cached_symbol *)malloc(sizeof(struct rebind_cached_symbol) * rebindings_nel);
  for (size_t i = 0; i < rebindings_nel; i++) {
    cached_rebindings.rebindings[i].name = rebindings[i].name;
    cached_rebindings.rebindings[i].replacement = rebindings[i].replacement;
    cached_rebindings.rebindings[i].replaced = rebindings[i].replaced;
  }
  cached_rebindings.next = NULL;
  int retval = rebind_symbols_for_image(&cached_rebindings, (const mach_header_t *)header, slide);
  free(cached_rebindings.rebindings);
  return retval;
}

__attribute__((visibility("hidden")))
int rebind_symbols(struct rebinding rebindings[], size_t rebindings_nel) {
  struct rebind_cached_symbol_list *new_node = (struct rebind_cached_symbol_list *)malloc(sizeof(struct rebind_cached_symbol_list));
  if (!new_node) {
    return -1;
  }
  new_node->rebindings = (struct rebind_cached_symbol *)malloc(sizeof(struct rebind_cached_symbol) * rebindings_nel);
  if (!new_node->rebindings) {
    free(new_node);
    return -1;
  }
  for (size_t i = 0; i < rebindings_nel; i++) {
    new_node->rebindings[i].name = rebindings[i].name;
    new_node->rebindings[i].replacement = rebindings[i].replacement;
    new_node->rebindings[i].replaced = rebindings[i].replaced;
  }
  new_node->next = _rebindings_head;
  _rebindings_head = new_node;

  static bool _dyld_registered = false;
  if (!_dyld_registered) {
    _dyld_registered = true;
    _dyld_register_func_for_add_image(_rebind_symbols_for_image);
  } else {
    uint32_t c = _dyld_image_count();
    for (uint32_t i = 0; i < c; i++) {
      _rebind_symbols_for_image(_dyld_get_image_header(i), _dyld_get_image_vmaddr_slide(i));
    }
  }
  return 0;
}
