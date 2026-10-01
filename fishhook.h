#ifndef FISHHOOK_H
#define FISHHOOK_H

#import <stddef.h>
#import <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct rebinding {
  const char *name;
  void *replacement;
  void **replaced;
};

__attribute__((visibility("hidden"))) int rebind_symbols(struct rebinding rebindings[], size_t rebindings_nel);
__attribute__((visibility("hidden"))) int rebind_symbols_image(void *header, intptr_t slide, struct rebinding rebindings[], size_t rebindings_nel);

#ifdef __cplusplus
}
#endif

#endif
