#![feature(alloc)]
#![feature(allocator_api)]
#![feature(allocator_internals)]
#![feature(compiler_builtins_lib)]
#![feature(const_cell_new)]
#![feature(const_fn)]
#![feature(const_ptr_null_mut)]
#![feature(global_allocator)]
#![feature(prelude_import)]
#![feature(proc_macro)]
#![feature(slice_get_slice)]
#![no_std]

extern crate alloc;
extern crate compiler_builtins;
extern crate drone_core;
extern crate drone_stm32 as drone_plat;
extern crate test;

#[prelude_import]
#[allow(unused_imports)]
use drone_plat::prelude::*;

use core::mem::size_of;
use drone_core::heap;

heap! {
  struct Heap;
  #[global_allocator]
  static ALLOC;
  size = 0;
  pools = [];
}

mod vtable {
  use drone_core::thr;
  use drone_plat::vtable;

  vtable! {
    pub struct Vtable1;
    #[allow(dead_code)]
    pub struct ThrIdx1;
    static THREADS1;
    extern struct Thr1;

    /// Test doc attribute
    #[doc = "test attribute"]
    pub NMI;
    /// Test doc attribute
    #[doc = "test attribute"]
    pub SYS_TICK;
    /// Test doc attribute
    #[doc = "test attribute"]
    pub 10: EXTI4;
    /// Test doc attribute
    #[doc = "test attribute"]
    pub 5: RCC;
  }

  vtable! {
    pub struct Vtable2;
    #[allow(dead_code)]
    pub struct ThrIdx2;
    static THREADS2;
    extern struct Thr2;
  }

  thr! {
    pub struct Thr1;
    extern static THREADS1;
  }

  thr! {
    pub struct Thr2;
    extern static THREADS2;
  }
}

#[test]
fn new() {
  unsafe extern "C" fn reset() -> ! {
    loop {}
  }
  vtable::Vtable1::new(reset);
  vtable::Vtable2::new(reset);
}

#[test]
fn size() {
  assert_eq!(
    (size_of::<vtable::Vtable1>() - size_of::<vtable::Vtable2>()) / 4,
    11
  );
}
