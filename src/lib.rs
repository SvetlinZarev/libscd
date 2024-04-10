#![no_std]
#![deny(unsafe_code)]

pub mod error;

#[cfg(feature = "sync")]
pub mod synchronous;

#[cfg(feature = "async")]
pub mod asynchronous;

pub(crate) mod internal;
